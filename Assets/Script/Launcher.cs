using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class Launcher : MonoBehaviour {

    public Texture[] TexBlock;

    AStar.Point GridSize = new AStar.Point(10, 10);
    List<AStar.Point> obstacles = new List<AStar.Point>();
    AStar.AStarRoutePlanner planner = new AStar.AStarRoutePlanner(10, 10, new AStar.SimpleCostGetter());

	// Use this for initialization
	void Start () 
    {
        Random.seed = 1111111;

        for (int i = 0; i < 12; ++i)
        {
            obstacles.Add(new AStar.Point(Random.Range(0, GridSize.X), Random.Range(0, GridSize.Y)));
        }
        planner.Initialize(obstacles);
	}

    float m_counter = 0;
    IList<AStar.Point> m_path = null;

	// Update is called once per frame
	void Update () 
    {
        m_counter += Time.deltaTime;
        if (m_counter > 0.5f)
        //if (Input.GetKeyDown(KeyCode.Space))
        {
            m_counter = 0;

            float t = Time.realtimeSinceStartup;
            //for (int i = 0; i < 10000; ++i)
            {
                AStar.Point s = GetRandPoint();
                AStar.Point e = GetRandPoint();

                //Debug.Log(s.ToString() + "   " + e.ToString());

                m_path = planner.Plan(s, e);
            }
            float diff = Time.realtimeSinceStartup - t;
            Debug.Log(diff.ToString());
            
        }
	}

    AStar.Point GetRandPoint()
    {
        while (true)
        {
            AStar.Point s = new AStar.Point(Random.Range(0, GridSize.X), Random.Range(0, GridSize.Y));
            if (!planner.Obstacles[s.X][s.Y])
                return s;
        }
    }

    bool PathContain(int x, int y)
    {
        if (m_path == null)
            return false;

        foreach (AStar.Point p in m_path)
        {
            if (p.X == x && p.Y == y)
                return true;
        }
        return false;
    }

    void OnGUI()
    {
        for (int y = 0; y < GridSize.Y; ++y)
        {
            for (int x = 0; x < GridSize.X; ++x)
            {
                int px = x;// -10;
                int py = y;// -10;

                Texture tex = null;
                if (planner.Obstacles[x][y])
                    tex = TexBlock[1];
                else if (PathContain(x, y))
                    tex = TexBlock[2];
                else
                    tex = TexBlock[0];
                GUI.Box(new Rect(px * 20, py * 20, 20, 20), tex);
            }
        }
    }

}
