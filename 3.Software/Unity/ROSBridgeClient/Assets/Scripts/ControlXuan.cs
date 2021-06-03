using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using InControl;
using RosSharp.RosBridgeClient;

public class ControlXuan : MonoBehaviour
{
    public XuanPublisher xuanPublisher;
    private InputDevice xboxController;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        xboxController = InputManager.ActiveDevice;

        xuanPublisher.SetServoAngle(xboxController.LeftStick.Value[0] * 90);
    }
}