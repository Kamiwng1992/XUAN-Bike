using UnityEngine;
using System.Collections;

public class ApplyImuPose : MonoBehaviour
{
    public bool useSlerp = false;

    Quaternion relaxQ, hQ, targetQ, lastQ;


    // Use this for initialization
    void Start()
    {
        relaxQ = this.transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        if (Serial._isUpdated)
        {
            targetQ = Quaternion.Inverse(hQ) *
                      new Quaternion(Serial._quat[1], Serial._quat[3], Serial._quat[2], -Serial._quat[0]);
            lastQ = this.transform.rotation;
            Serial._isUpdated = false;
        }

        if (useSlerp)
            transform.rotation = Quaternion.Slerp(lastQ, targetQ, Time.deltaTime * 20);
        else
            transform.localRotation = targetQ;

        if (Input.GetKeyDown(KeyCode.Space))
        {
            hQ = new Quaternion(Serial._quat[1], Serial._quat[3], Serial._quat[2],
                -Serial._quat[0]) * Quaternion.Inverse(relaxQ);
        }
    }
}