using System;
using System.Collections.Generic;
using UnityEngine;

public class SceneController : MonoBehaviour
{
    public float g = 9.8f; // Gravité
    public Vector3 Y = new Vector3(0, -1, 0);
    public GameObject pointsContainer;
    public List<GameObject> listPoint;
    public GameObject pointPrefab;
    public int nbrPoint;
    public float x;
    public float y;
    public float k;
    public float knear;
    public float h;
    public float massMin;
    public float massMax;
    public float r0;
    public float rnear;
    public float pnear;
    public float foutside;

    // Start is called before the first frame update
    void Start()
    {
        CreatePoint();
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawLine(new Vector3(0, 0), new Vector3(x,0));
        Debug.DrawLine(new Vector3(0, 0), new Vector3(0,y));
        Debug.DrawLine(new Vector3(x, 0), new Vector3(x,y));
        Debug.DrawLine(new Vector3(0, y), new Vector3(x,y));


        // Spatial hashing
        SpatialHash2D sh2d = new SpatialHash2D(x, y, h);
        foreach (GameObject point in listPoint)
        {
            sh2d.Insert(point);
        }

        PVFS(listPoint, sh2d, g, Y, x, y, k, knear, pnear, rnear, h, r0, foutside);
    }

    #region Fonction Calcules

    public static Vector3 CalculAcceleration(float m, float g, float Cd, Vector3 Z, Vector3 V)
    {
        return (m * g * Z - V * Cd * V.magnitude) / m;
    }

    private static Vector3 Vdt(float m, float g, float cd, Vector3 Z, Vector3 V)
    {
        return V + Time.fixedDeltaTime * g * Z;
    }

    private static Vector3 Pdt(Vector3 V, Vector3 P)
    {
        return P + V * Time.fixedDeltaTime;
    }

    #endregion

    public void CreatePoint()
    {
        for (int i = 0; i < nbrPoint; i++)
        {
            GameObject instance = Instantiate<GameObject>(pointPrefab, pointsContainer.transform);
            instance.transform.position = new Vector3(UnityEngine.Random.Range(0, x), UnityEngine.Random.Range(0, y));
            Point point = instance.GetComponent<Point>();
            point.m = 80;
            point.cd = 1f;
            point.h = h;
            point.m = UnityEngine.Random.Range(massMin, massMax);

            Color low = Color.blue;
            Color high = Color.red;
            float rangedMass = (((point.m - massMin)) / (massMax - massMin));
            Color current = Color.Lerp(low, high, rangedMass);
            instance.GetComponent<SpriteRenderer>().color = current;
            listPoint.Add(instance);
        }
    }


    public static void PVFS(List<GameObject> listPoint, SpatialHash2D sh2d, float g, Vector3 Y, float boundaryX, float boundaryY, float k, float knear, float pnear, float rnear, float h, float r0, float foutside)
    {
        foreach (GameObject item in listPoint)
        {
            // Apply gravity
            Point point = item.GetComponent<Point>();
            Transform transform = item.GetComponent<Transform>();
            point.pos = transform.position;

            point.v = Vdt(point.m, g, point.cd, Y, point.v);

            // save previous position
            point.prevPos = point.pos;

            // advance to predicted position
            point.pos = Pdt(point.v, point.pos);
            transform.position = point.pos;


            // Clamp
            if (point.pos.y < 0)
            {
                point.pos += Vector3.up * (new Vector3(point.pos.x, 0) - point.pos).magnitude * foutside * (Time.fixedDeltaTime * Time.fixedDeltaTime) / point.m;
            }
            if (point.pos.y > boundaryY)
            {
                point.pos += Vector3.down * (new Vector3(point.pos.x, boundaryY) - point.pos).magnitude * foutside * (Time.fixedDeltaTime * Time.fixedDeltaTime) / point.m;
            }
            if (point.pos.x < 0)
            {
                point.pos += Vector3.right * (new Vector3(0, point.pos.y) - point.pos).magnitude * foutside * (Time.fixedDeltaTime * Time.fixedDeltaTime) / point.m;
            }
            if (point.pos.x > boundaryX)
            {
                point.pos += Vector3.left * (new Vector3(boundaryX, point.pos.y) - point.pos).magnitude * foutside * (Time.fixedDeltaTime * Time.fixedDeltaTime) / point.m;
            }

        }

        // modify velocities with pairwise viscosity impulses
        //  applyViscosity

        // add and remove springs, change rest lengths
        //  adjustSprings

        // modify positions according to springs,
        // double density relaxation, and collisions

        //  applySpringDisplacements

        //  doubleDensityRelaxation
        foreach (GameObject i in listPoint)
        {
            Point pointI = i.GetComponent<Point>();
            Transform transformI = i.GetComponent<Transform>();
            float r = 0;
            float p;
            float q;
            Vector3 dirIJ;
            // compute density and near-density

            List<GameObject> neighborhood = sh2d.GetNearby(i);

            foreach (GameObject j in neighborhood)
            {
                Point pointJ = j.GetComponent<Point>();
                dirIJ = pointJ.pos - pointI.pos;
                q = dirIJ.magnitude / h;

                if (q < 1)
                {
                    r += Mathf.Pow(1 - q, 2);
                    rnear += Mathf.Pow(1 - q, 3);
                }
            }

            // compute pressure and near-pressure

            p = k * (r - r0);
            pnear = knear * rnear;

            Vector3 dx = Vector3.zero;

            foreach (GameObject j in neighborhood)
            {
                Point pointJ = j.GetComponent<Point>();
                Transform transformJ = j.GetComponent<Transform>();
                dirIJ = pointJ.pos - pointI.pos;
                q = dirIJ.magnitude / h;

                if (q < 1)
                {
                    Vector3 D = (Time.fixedDeltaTime * Time.fixedDeltaTime) / pointI.m * (p * (1 - q) + pnear * Mathf.Pow(1 - q, 2)) * dirIJ.normalized;
                    transformJ.position = pointJ.pos + D / 2;
                    dx = dx - D / 2;
                }
            }

            transformI.position = pointI.pos + dx;
        }

        //resolveCollisions

        // use previous position to compute next velocity
        foreach (GameObject item in listPoint)
        {
            Point pointI = item.GetComponent<Point>();
            pointI.v = (pointI.pos - pointI.prevPos) / Time.fixedDeltaTime;
        }

    }

    public static List<GameObject> GetNeighborhood(GameObject pointCentre, List<GameObject> listPoint)
    {
        List<GameObject> listNeighborghs = new List<GameObject>();

        Point pointI = pointCentre.GetComponent<Point>();
        Transform transformI = pointCentre.GetComponent<Transform>();

        foreach (GameObject j in listPoint)
        {
            Point pointJ = j.GetComponent<Point>();
            Transform transformJ = j.GetComponent<Transform>();
            float distIJ = (transformJ.position - transformI.position).magnitude;
            if (distIJ <= pointI.h)
            {
                listNeighborghs.Add(j);
            }
        }

        return listNeighborghs;
    }
}

public class SpatialHash2D
{
    public float sceneWidth;
    public float sceneHeight;
    public float cellSize;

    //Buckets: store actual bullets
    public Dictionary<Tuple<int, int>, List<GameObject>> buckets;
    //Max bucket size, avoid calculated bucket out of range bug (see below)
    public int bucketSize;
    //Init the grid
    public SpatialHash2D(float sceneWidth, float sceneHeight, float cellSize)
    {
        this.buckets = new Dictionary<Tuple<int, int>, List<GameObject>>();
        this.cellSize = cellSize;
        this.bucketSize = buckets.Count;
    }

    //Insert an Object to the bucket
    public void Insert(GameObject obj)
    {
        Tuple<int, int> cellIDs = GetBucketIDs(obj);
        if (!buckets.ContainsKey(cellIDs))
        {
            this.buckets.Add(cellIDs, new List<GameObject>());
        }
        this.buckets[cellIDs].Add(obj);
    }

    // Get the Dictionary Keys of Buckets that may contain bullets that OVERLAPS obj
    private Tuple<int, int> GetBucketIDs(GameObject obj)
    {
        Vector3Int vecInt = Vector3Int.FloorToInt(obj.transform.position);
        Tuple<int, int> bucketIDs = Tuple.Create(vecInt.x, vecInt.y);

        return bucketIDs;
    }

    public List<GameObject> GetNearby(GameObject obj)
    {
        List<GameObject> objects = new List<GameObject>();

        Tuple<int, int> cellIDs = GetBucketIDs(obj);

        for (int y = cellIDs.Item2 - 1; y <= cellIDs.Item2 + 1; y++)
        {
            for (int x = cellIDs.Item1 - 1; x <= cellIDs.Item1 + 1; x++)
            {
                if (buckets.TryGetValue(Tuple.Create(x, y), out List<GameObject> values))
                {
                    objects.AddRange(values);
                }
            }
        }
        return objects;
    }
}
