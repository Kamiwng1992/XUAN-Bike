using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomShake : MonoBehaviour
{
    // Start is called before the first frame update
    private Quaternion start, end;
    private int tick;

    void Start()
    {
        start = transform.localRotation;
    }

    // Update is called once per frame
    void Update()
    {
        tick++;
        end = start * Quaternion.Euler(new Vector3(0, 0, Random.Range(-1, 1)));

        transform.localRotation = Quaternion.Slerp(transform.localRotation, end, 0.5f);

        transform.localPosition -= new Vector3(0, 0, 0.01f + 0.001f * Random.Range(-1, 1));
    }
}