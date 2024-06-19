using System.Collections.Generic;

using asim.unity.generation.maze;
using asim.unity.managers.tilemanager;
using asim.unity.pathfinding;

using UnityEngine;

/// <summary>
/// Uses asim maze generation to generate a maze to be used to showcase pathfinding/maze traversal
/// </summary>
public class Maze_Traversal : MonoBehaviour
{
    [SerializeField] SimpleTile_TileManager Tilemanager;
    [SerializeField] GameObject StartGO;
    [SerializeField] GameObject GoalGO;
    [SerializeField] LineRenderer lineRenderer;

    public int MazeRowSize = 10;
    public int MazeColSize = 10;

    char[][] maze;
    void Start()
    {
        maze = GenerateRandomMaze(MazeRowSize, MazeColSize);
    }
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            maze = GenerateRandomMaze(MazeRowSize, MazeColSize);
        }
        if (Input.GetKeyDown(KeyCode.R) || Input.GetKeyDown(KeyCode.Space))
        {
            //Convert maze from a char array to bool array
            bool[][] grid = convertMaze(maze);
            Vector2 start = new Vector2(Mathf.Round(StartGO.transform.position.x), Mathf.Round(StartGO.transform.position.y));
            Vector2 goal = new Vector2(Mathf.Round(GoalGO.transform.position.x), Mathf.Round(GoalGO.transform.position.y));

            List<Vector2> path = CalculateAStarPath(grid, start, goal);
            
            if (path != null && path.Count != 0)
            {
                //OptimisePath(path);

                Vector3[] linepos = new Vector3[path.Count];
                for (int i = 0; i < path.Count; i++)
                {
                    linepos[i] = path[i];
                }

                lineRenderer.positionCount = linepos.Length;
                lineRenderer.SetPositions(linepos);
            }
        }
    }
    char[][] GenerateRandomMaze(int rows, int cols)
    {
        RandomIterativeDFSMaze maze = new RandomIterativeDFSMaze();
        maze.CreateMaze(rows, cols);

        Tilemanager.Removetiles();
        GameObject go = Tilemanager.SpawnTilesetFromCharArray(maze.GetMaze());
        go.transform.localScale = new Vector3(1, -1, 1);

        return maze.GetMaze();
    }
    List<Vector2> CalculateAStarPath(bool[][] maze,Vector2 start,Vector2 goal)
    {
        List<Vector2> GridDirections = new List<Vector2>
        {
            Vector2.up,
            Vector2.down,
            Vector2.left,
            Vector2.right
        };
        //GridDirections.Add(new Vector2(1,1));
        //GridDirections.Add(new Vector2(-1, -1));
        //GridDirections.Add(new Vector2(-1, 1));
        //GridDirections.Add(new Vector2(1, -1));

        AStar astar = new AStar(GridDirections);
        return astar.FindClosestPath(maze, start, goal);
    }

    bool[][] convertMaze(char[][] maze)
    {
        bool[][] boolarray = new bool[maze.Length][];
        for (int i = 0; i < maze.Length; i++)
        {
            boolarray[i] = new bool[maze[i].Length];
            for (int j = 0; j < maze[i].Length; j++)
            {
                boolarray[i][j] = maze[i][j] == '#' ? false : true;
            }
        }
        return boolarray;
    }
    void OptimisePath(List<Vector2> path)
    {
        Vector2 lastdiff = Vector2.zero;
        Vector2 currenntdiff = Vector2.zero;

        if (path == null) return;
        if (path.Count > 1)
        {
            for (int i = 1; i < path.Count; i++)
            {
                currenntdiff = path[i] - path[i - 1];
                if (currenntdiff == lastdiff)
                {
                    path.RemoveAt(i - 1); i--;
                }
                else
                {
                    lastdiff = currenntdiff;
                }
            }
        }
    }
}
