using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreatePoint : MonoBehaviour
{
    public GameObject pointsContainer;
    public GameObject pointPrefab;
    public int nbrPoint;
    public float x;
    public float y;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < nbrPoint; i++)
        {
            GameObject instance = Instantiate<GameObject>(pointPrefab, pointsContainer.transform);
            instance.transform.position = new Vector3(Random.Range(-x, x), Random.Range(-y, y));
            instance.GetComponent<SpriteRenderer>().color = Random.ColorHSV();

        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
