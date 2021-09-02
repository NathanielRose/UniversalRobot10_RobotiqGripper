using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10MoveitConfig;
using UnityEngine;
using UnityEngine.SceneManagement;

using ROSGeometry;
using RosImage = RosMessageTypes.Sensor.Image;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;
using Transform = UnityEngine.Transform;
using Random = UnityEngine.Random;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using System.Collections.Generic;

public class UR10TrajectoryPlanner : MonoBehaviour
{
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    private GameObject experimentLogic;

    // ROS Connector
    ROSConnection ros;
    private int numRobotJoints = 6;
    public string topicName = "SourceDestination_input";

    // Publish Toggle
    public bool publisher_available = true; 

    // Publish the UR10 position and rotation every N seconds
    public float publishMessageFrequency = 0.9f;

    // Hardcoded variables 
    private readonly float jointAssignmentWait = 0.06f;
    private readonly float poseAssignmentWait = 0.5f;
    private readonly float gripperAngle = 15.5f;
    private readonly Vector3 pickPoseOffset = new Vector3(0, 0.24f, 0);
    private readonly Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);
    // Multipliers correspond to the URDF mimic tag for each joint
    //private float[] multipliers = new float[] { -0f, -1.8f, -0f, 0f, 1.8f, 0f };
    private float[] multipliers = new float[] { -0f, -1.8f, -0f, 0f, 1.8f, 0f };

    // Assures that the gripper is always positioned above the target cube before grasping.
    //private readonly Quaternion pickOrientation = Quaternion.Euler(90, 90, 0);
    private readonly Quaternion pickOrientation = new Quaternion(-0.5f, -0.5f, 0.5f, -0.5f);

    // Variables required for ROS communication
    public string rosServiceName = "ur10_moveit";

    public GameObject ur10;
    public GameObject target;
    public GameObject targetPlacement;

    // For Game Logic and Data Modeling
    public GameObject experimentDataManager;
    private ExperimentDataManager experimentData;
    public bool beginExecutionTimer = false;
    public float rosExecutionTimer = 0.0f;
    public float articulationsExecutionTimer = 0.0f;
    public float publishJointsTimer = 0.0f;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    ArticulationBody[] articulationChain;
    private List<ArticulationBody> gripperJoints;
    private ArticulationBody leftGripper;
    private ArticulationBody rightGripper;

    private Transform gripperBase;
    private Transform leftGripperGameObject;
    private Transform rightGripperGameObject;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    };

    /// <summary>
    ///     Opens and closes the attached gripper tool based on a gripping angle.
    /// </summary>
    /// <param name="toClose"></param>
    /// <returns></returns>
    public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle: 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            gripperJoints[i].xDrive = curXDrive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR10MoveitJoints</returns>
    UR10MoveItJoints CurrentJointConfig()
    {
        UR10MoveItJoints joints = new UR10MoveItJoints();
        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;

        return joints;
    }


    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        beginExecutionTimer = true;
        UR10MoverServiceRequest request = new UR10MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (target.transform.position + pickPoseOffset).To<FLU>(),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (targetPlacement.transform.position + placePoseOffset).To<FLU>(),
            orientation = pickOrientation.To <FLU>()
        };
        ros.SendServiceMessage<UR10MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(UR10MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///         PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// 
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    private IEnumerator ExecuteTrajectories(UR10MoverServiceResponse response)
    {
        experimentData.trialData.rosExecution_time = timeElapsed - publishJointsTimer;
        experimentData.trialData.target_position = target.transform.position;
        string current_scene = SceneManager.GetActiveScene().name;
        Debug.Log("This is the scene: " + current_scene);
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(jointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    if (current_scene == "UR10GripScene" || current_scene == "UR10VRScene")

                    {
                        //CloseGripper();
                        string hand_link = "world/base_link/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link" + "/wrist_3_link";
                        StartCoroutine(IterateToGrip(true));
                        yield return new WaitForSeconds(jointAssignmentWait);
                        //target.transform.parent = ur10.transform.Find(hand_link).transform;

                    }
                    else
                    {
                        Debug.Log("Simulate CLOSE Gripper");
                        string hand_link = "world/base_link/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link" + "/wrist_3_link";
                        target.transform.parent = ur10.transform.Find(hand_link).transform;
                        Debug.Log(ur10.transform.Find(hand_link).transform);
                    }
                }
                else if(poseIndex == (int)Poses.Place)
                {
                    // All trajectories have been executed, open the gripper to place the target cube
                    if (current_scene == "UR10GripScene" | current_scene == "UR10VRScene")
                    //if (current_scene == "UR10GripScene" )
                    {
                        //OpenGripper();
                        yield return new WaitForSeconds(poseAssignmentWait);
                        // Open the gripper to place the target cube
                        //target.transform.parent = null;
                        StartCoroutine(IterateToGrip(false));
                        experimentData.trialData.articulationsExecution_time = timeElapsed - articulationsExecutionTimer;
                        experimentData.trialData.placedObject_position = target.transform.position;
                        experimentData.trialData.targetPlacement_position = targetPlacement.transform.position;
                        timeElapsed = 0.0f;
                        beginExecutionTimer = false;
                        experimentData.writeFile();
                        publisher_available = true;
                    }
                    else
                    {
                        yield return new WaitForSeconds(poseAssignmentWait);
                        Debug.Log("Simulate OPEN Gripper");
                        target.transform.parent = null;
                    }
                }
                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(poseAssignmentWait);
            }
            // Re-enable buttons
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.instance;

        // Adjust UR10 into a default position from Ali: [0, -2.79, 2.356, -1.134, -1.57, 0] --> In Radians
        var joint1XDrive = jointArticulationBodies[1].xDrive;
        joint1XDrive.target = -160;
        jointArticulationBodies[1].xDrive = joint1XDrive;

        var joint2XDrive = jointArticulationBodies[2].xDrive;
        joint2XDrive.target = 135;
        jointArticulationBodies[2].xDrive = joint2XDrive;

        var joint3XDrive = jointArticulationBodies[3].xDrive;
        joint3XDrive.target = -65;
        jointArticulationBodies[3].xDrive = joint3XDrive;

        var joint4XDrive = jointArticulationBodies[4].xDrive;
        joint4XDrive.target = -90;
        jointArticulationBodies[4].xDrive = joint4XDrive;

        experimentData = experimentDataManager.GetComponent<ExperimentDataManager>();
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Awake()
    {
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = ur10.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string upper_arm_link = shoulder_link + "/upper_arm_link";
        jointArticulationBodies[1] = ur10.transform.Find(upper_arm_link).GetComponent<ArticulationBody>();

        string forearm_link = upper_arm_link + "/forearm_link";
        jointArticulationBodies[2] = ur10.transform.Find(forearm_link).GetComponent<ArticulationBody>();

        string wrist_1_link = forearm_link + "/wrist_1_link";
        jointArticulationBodies[3] = ur10.transform.Find(wrist_1_link).GetComponent<ArticulationBody>();

        string wrist_2_link = wrist_1_link + "/wrist_2_link";
        jointArticulationBodies[4] = ur10.transform.Find(wrist_2_link).GetComponent<ArticulationBody>();

        string hand_link = wrist_2_link + "/wrist_3_link";
        jointArticulationBodies[5] = ur10.transform.Find(hand_link).GetComponent<ArticulationBody>();

        // ToDo: Finish the left and right finger joints and assign them
        articulationChain = ur10.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();

        var gripperJointNames = new string[] { "right_outer_knuckle", "right_inner_finger", "right_inner_knuckle", "left_outer_knuckle", "left_inner_finger", "left_inner_knuckle" };
        gripperJoints = new List<ArticulationBody>();

        foreach (ArticulationBody articulationBody in ur10.GetComponentsInChildren<ArticulationBody>())
        {
            if (gripperJointNames.Contains(articulationBody.name))
            {
                gripperJoints.Add(articulationBody);
            }
        }

    }

    // Update is called once per frame
    void Update()
    {
        if (beginExecutionTimer)
        {
            timeElapsed += Time.deltaTime;
        }
    }

}
