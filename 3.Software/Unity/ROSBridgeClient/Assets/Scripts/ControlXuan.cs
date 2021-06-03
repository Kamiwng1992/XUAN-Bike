using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using InControl;
using RosSharp.RosBridgeClient;

public class ControlXuan : MonoBehaviour
{
    public XuanPublisher xuanPublisher;
    private InputDevice xboxController;

    private int tick;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        tick++;
        if (tick % 10 == 0)
        {
            xboxController = InputManager.ActiveDevice;
            xuanPublisher.SetMove(xboxController.LeftStick.Value[0] * 100, xboxController.LeftStick.Value[1] * 100);
        }
    }
}