using UnityEngine;
using System.Collections;
using System.IO.Ports;

//using UnityEditor;

public class Serial : MonoBehaviour
{
    public static SerialPort IMU_port;
    public static float[] _quat = new float[4];
    public static bool _isUpdated = false;

    static int cnt = 0;

    // Use this for initialization
    void Awake()
    {
        OpenConnection();
    }


    void FixedUpdate()
    {
        try
        {
            string value = IMU_port.ReadLine();
            string[] recv = new string[4];

            //IMU_port.BaseStream.Flush ();
            if (value != null)
            {
                recv = value.Split(',');
                for (int i = 0; i < 4; i++)
                    _quat[i] = float.Parse(recv[i]);
                _isUpdated = true;
            }
        }
        catch (System.Exception ex)
        {
            Debug.LogException(ex);
        }
    }

    public void OpenConnection()
    {
        if (SerialPort.GetPortNames().Length > 0)
            IMU_port = new SerialPort(SerialPort.GetPortNames()[0], 115200, Parity.None, 8, StopBits.One);
        if (IMU_port != null)
        {
            if (IMU_port.IsOpen)
            {
                IMU_port.Close();
                Debug.Log("Closing port, because it was already open!");
            }
            else
            {
                IMU_port.Open(); // opens the connection
                IMU_port.ReadTimeout = 1; // sets the timeout value before reporting error
                Debug.Log("Port Opened!");
            }
        }
        else
        {
            Debug.Log("Port == null");
            Application.Quit();
            //EditorApplication.isPlaying = false;
            return;
        }
    }

    void OnApplicationQuit()
    {
        IMU_port.Close();
    }
}