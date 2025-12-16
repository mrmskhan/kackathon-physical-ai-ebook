using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// Unity Scene Setup for ROS 2 Integration
/// 
/// This script:
/// 1. Initializes ROS-TCP-Connector
/// 2. Subscribes to common ROS 2 topics
/// 3. Visualizes robot pose and sensor data
/// 
/// Usage:
/// 1. Attach to an empty GameObject in your Unity scene
/// 2. Configure ROS IP and port in Robotics > ROS Settings
/// 3. Press Play to start receiving ROS 2 messages
/// </summary>
public class UnitySceneSetup : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    [Tooltip("ROS 2 IP address (Ubuntu machine)")]
    public string rosIPAddress = "192.168.1.100";
    
    [Tooltip("ROS 2 TCP port")]
    public int rosPort = 10000;
    
    [Header("Topic Subscriptions")]
    [Tooltip("Subscribe to /cmd_vel for robot velocity commands")]
    public bool subscribeToCmdVel = true;
    
    [Tooltip("Subscribe to /lidar for laser scan data")]
    public bool subscribeToLidar = false;
    
    [Tooltip("Subscribe to /camera/image for camera feed")]
    public bool subscribeToCamera = false;
    
    [Header("Visualization")]
    [Tooltip("GameObject representing the robot (will move based on /cmd_vel)")]
    public GameObject robotObject;
    
    [Tooltip("UI RawImage for displaying camera feed")]
    public UnityEngine.UI.RawImage cameraDisplay;
    
    private ROSConnection ros;
    private Texture2D cameraTexture;
    
    // Robot movement state
    private Vector3 linearVelocity;
    private Vector3 angularVelocity;
    
    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.ConnectOnStart = true;
        
        Debug.Log($"Connecting to ROS 2 at {rosIPAddress}:{rosPort}");
        
        // Subscribe to topics
        if (subscribeToCmdVel)
        {
            ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveCmdVel);
            Debug.Log("Subscribed to /cmd_vel");
        }
        
        if (subscribeToLidar)
        {
            ros.Subscribe<LaserScanMsg>("/lidar", ReceiveLidarScan);
            Debug.Log("Subscribed to /lidar");
        }
        
        if (subscribeToCamera)
        {
            ros.Subscribe<ImageMsg>("/camera/image", ReceiveCameraImage);
            Debug.Log("Subscribed to /camera/image");
        }
        
        // Validate robot object
        if (robotObject == null)
        {
            Debug.LogWarning("Robot object not assigned. Create a GameObject and assign it in the Inspector.");
        }
    }
    
    void Update()
    {
        // Update robot position based on velocity commands
        if (robotObject != null && subscribeToCmdVel)
        {
            // Linear movement (forward/backward)
            float linearX = (float)linearVelocity.x;
            robotObject.transform.Translate(Vector3.forward * linearX * Time.deltaTime);
            
            // Angular movement (rotation)
            float angularZ = (float)angularVelocity.z;
            robotObject.transform.Rotate(Vector3.up, angularZ * Mathf.Rad2Deg * Time.deltaTime);
        }
    }
    
    /// <summary>
    /// Callback for /cmd_vel topic (robot velocity commands)
    /// </summary>
    void ReceiveCmdVel(TwistMsg msg)
    {
        // Store velocity for Update() to apply
        linearVelocity = new Vector3(
            (float)msg.linear.x,
            (float)msg.linear.y,
            (float)msg.linear.z
        );
        
        angularVelocity = new Vector3(
            (float)msg.angular.x,
            (float)msg.angular.y,
            (float)msg.angular.z
        );
        
        Debug.Log($"Cmd Vel: linear={linearVelocity.x:F2}, angular={angularVelocity.z:F2}");
    }
    
    /// <summary>
    /// Callback for /lidar topic (laser scan data)
    /// </summary>
    void ReceiveLidarScan(LaserScanMsg msg)
    {
        // Lidar data visualization (example: draw debug rays)
        float angleMin = msg.angle_min;
        float angleIncrement = msg.angle_increment;
        
        for (int i = 0; i < msg.ranges.Length; i += 10) // Sample every 10th ray for performance
        {
            float angle = angleMin + (i * angleIncrement);
            float range = msg.ranges[i];
            
            if (range >= msg.range_min && range <= msg.range_max)
            {
                Vector3 direction = new Vector3(
                    Mathf.Cos(angle),
                    0,
                    Mathf.Sin(angle)
                );
                
                Vector3 hitPoint = robotObject.transform.position + direction * range;
                Debug.DrawLine(robotObject.transform.position, hitPoint, Color.red, 0.1f);
            }
        }
    }
    
    /// <summary>
    /// Callback for /camera/image topic (RGB camera feed)
    /// </summary>
    void ReceiveCameraImage(ImageMsg msg)
    {
        if (cameraDisplay == null)
        {
            Debug.LogWarning("Camera display RawImage not assigned");
            return;
        }
        
        // Initialize texture on first message
        if (cameraTexture == null)
        {
            cameraTexture = new Texture2D((int)msg.width, (int)msg.height, TextureFormat.RGB24, false);
            cameraDisplay.texture = cameraTexture;
        }
        
        // Update texture with new image data
        cameraTexture.LoadRawTextureData(msg.data);
        cameraTexture.Apply();
    }
    
    void OnApplicationQuit()
    {
        // Clean up ROS connection
        if (ros != null)
        {
            Debug.Log("Disconnecting from ROS 2");
        }
    }
    
    void OnGUI()
    {
        // Display connection status in top-left corner
        GUIStyle style = new GUIStyle();
        style.fontSize = 16;
        style.normal.textColor = ros != null && ros.HasConnectionThread ? Color.green : Color.red;
        
        string status = ros != null && ros.HasConnectionThread ? "Connected" : "Disconnected";
        GUI.Label(new Rect(10, 10, 200, 30), $"ROS 2: {status}", style);
    }
}
