using System.Collections.Generic;

using UnityEngine;

/// <summary>
/// A* Search Based on Psudocode from https://en.wikipedia.org/wiki/A*_search_algorithm
/// 2D grid map
/// </summary>
namespace asim.unity.pathfinding
{
    public class AStar 
    {
        //TODO minor optimise of node struct , such that reconstruct_path uses a directional list instead of node last node refrence
        internal class Node
        {
            public Vector2 Pos;
            public float GCost;
            public float FCost;
            public Node CameFrom;
        }

        const float errorIndex = -100;

        List<Vector2> gridDirections;

        public enum HeuristicAlgoType
        {
            Euclidian,
            Manhattan,
            Chebyshev
        }

        HeuristicAlgoType HCostAlgo;
        List<Node> closedset;
        List<Node> openset;

        public AStar(List<Vector2> inGridDirections, HeuristicAlgoType inalgo = HeuristicAlgoType.Euclidian)
        {
            gridDirections = inGridDirections;
            HCostAlgo = inalgo;
        }

        //NOTE, this is only for non negative value position, that means origin starts at bottom left
        public List<Vector2> FindClosestPath(bool[][] GridData, Vector2 Start, Vector2 End)
        {
            closedset = new List<Node>();
            openset = new List<Node>();

            if (GridData == null)
                return null;
            if (Start.x < 0 || Start.x >= GridData[0].Length
                || Start.y < 0 || Start.y >= GridData.Length
                || End.x < 0 || End.x >= GridData[0].Length
                || End.y < 0 || End.y >= GridData.Length)
                return null; //Theres no Path from and to an invalid node(negative val or exceed GridData,rows,cols))
            if (GridData[(int) Start.y][(int) Start.x] == false)
            {
                Start = findClosesNeighbour(GridData, Start, 3); //Set Start pos as closest avaiable pos
                if (Start == new Vector2(errorIndex, errorIndex))
                    return null;
            }
            if (GridData[(int) End.y][(int) End.x] == false)
            {
                End = findClosesNeighbour(GridData, End, 3); //Set end pos as closest avaiable pos
                if (End == new Vector2(errorIndex, errorIndex))
                    return null;
            }

            /*Add First Node To Openset*/
            Node startNode = new Node();
            startNode.Pos = Start;
            startNode.GCost = 0;
            startNode.FCost = CalculatFCost(startNode.Pos, End, HCostAlgo);
            startNode.CameFrom = null;
            openset.Add(startNode);

            Node lowestfCostNode;
            while (openset.Count > 0)
            {
                lowestfCostNode = getLowestFCost();

                if (lowestfCostNode.Pos == End)
                    return reconstruct_path(lowestfCostNode); //reconstruct path, returns a list of backtracked nodes

                openset.Remove(lowestfCostNode);
                closedset.Add(lowestfCostNode);

                foreach (Vector2 dir in gridDirections)
                {
                    /*Find All Possible Neighbour Nodes(passable nodes)*/
                    Vector2 Neighbourpos = lowestfCostNode.Pos + dir;

                    if (Neighbourpos.x < 0 || Neighbourpos.x >= GridData[0].Length
                        || Neighbourpos.y < 0 || Neighbourpos.y >= GridData.Length)
                        continue; //Theres no position that is out of gridmap(negative val or exceed GridData,rows,cols)

                    if (GridData[(int) Neighbourpos.y][(int) Neighbourpos.x] == false)
                        continue; //Ignore the Neighbour Nodes that is inpassable - according to GridData

                    Node Neighbour = findNeighbourinSet(closedset, Neighbourpos);
                    if (Neighbour != null)
                        continue; //Ignore the neighbor which is already evaluated(in Closedset)

                    /*Calculate the tentative gCost of neightbour(start -> neighbour)*/
                    float gCost = lowestfCostNode.GCost + Vector2.Distance(lowestfCostNode.Pos, Neighbourpos);

                    Neighbour = findNeighbourinSet(openset, Neighbourpos);
                    if (Neighbour == null) //Discoved a new neightbour,add new node
                    {
                        Node newNode = new Node();
                        newNode.Pos = Neighbourpos;
                        newNode.GCost = gCost;
                        newNode.FCost = CalculatFCost(newNode.Pos, End, HCostAlgo);
                        newNode.CameFrom = lowestfCostNode;
                        openset.Add(newNode);
                    }
                    else if (gCost >= Neighbour.GCost)
                        continue; // This is not a better path.
                    else
                    {
                        /*This new path is better(lower g cost), Update it*/
                        Neighbour.GCost = gCost;
                        Neighbour.FCost = gCost + CalculatFCost(Neighbour.Pos, End, HCostAlgo);
                        Neighbour.CameFrom = lowestfCostNode;
                    }
                }
            }
            return null;
        }

        Vector2 findClosesNeighbour(bool[][] GridData, Vector2 startpos, int maxSearchSize)
        {
            List<Vector2> Closedset = new List<Vector2>();
            List<Vector2> Openset = new List<Vector2>();

            Openset.Add(startpos);
            int currentSize = 1;
            int neighboursLeftToIncreaseSize = 0;
            int neighboursInSize = 0;

            while (Openset.Count > 0 && currentSize <= maxSearchSize)
            {
                foreach (Vector2 dir in gridDirections)
                {
                    Vector2 neighbourpos = Openset[0] + dir;

                    if (neighbourpos.x < 0 || neighbourpos.x >= GridData[0].Length
                        || neighbourpos.y < 0 || neighbourpos.y >= GridData.Length)
                        continue; //Theres no position that is out of gridmap(negative val or exceed GridData,rows,cols)
                    if (GridData[(int) neighbourpos.y][(int) neighbourpos.x] == true)
                        return neighbourpos;
                    else
                    {
                        if (isNeighbourinSet(Closedset, neighbourpos) == false)
                        {
                            if (isNeighbourinSet(Openset, neighbourpos) == false)
                            {
                                Openset.Add(neighbourpos);
                                neighboursInSize++;
                            }
                        }
                    }
                }
                Closedset.Add(Openset[0]);
                Openset.RemoveAt(0);

                neighboursLeftToIncreaseSize--;
                if (neighboursLeftToIncreaseSize <= 0)
                {
                    neighboursLeftToIncreaseSize = neighboursInSize;
                    neighboursInSize = 0;
                    currentSize++;
                }
            }
            return new Vector2(errorIndex, errorIndex);
        }

        /// <summary>
        /// Returns the node with the lowest fcost in Openset
        /// </summary>
        Node getLowestFCost()
        {
            int lowestfcostindex = 0;
            for (int i = 1; i < openset.Count; i++)
            {
                if (openset[i].FCost < openset[lowestfcostindex].FCost)
                    lowestfcostindex = i;
            }
            return openset[lowestfcostindex];
        }

        /// <summary>
        /// Returns the node in the list, with matching position
        /// </summary>
        Node findNeighbourinSet(List<Node> list, Vector2 pos)
        {
            //Check if theres an Existing Node occupying Pos
            foreach (Node node in list)
            {
                if (node.Pos == pos)
                    return node;
            }
            return null;
        }

        bool isNeighbourinSet(List<Vector2> list, Vector2 pos)
        {
            //Check if theres an Existing Node occupying Pos
            foreach (Vector2 v in list)
            {
                if (v == pos)
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Calculate the FCost
        /// </summary>
        float CalculatFCost(Vector2 Start, Vector2 Target, HeuristicAlgoType AlgoType)
        {
            float x = Target.x - Start.x;
            float y = Target.y - Start.y;
            float HCost = 0;

            switch (AlgoType)
            {
                case HeuristicAlgoType.Euclidian:
                    HCost = Mathf.Sqrt(x * x + y * y);
                    break;
                case HeuristicAlgoType.Manhattan:
                    HCost = Mathf.Abs(x) + Mathf.Abs(y);
                    break;
                case HeuristicAlgoType.Chebyshev:
                    HCost = Mathf.Max(Mathf.Abs(x), Mathf.Abs(y));
                    break;
            }
            return HCost;
        }

        /// <summary>
        /// Returns a list of backtracked nodes in order of startNode to TargetNode
        /// </summary>
        List<Vector2> reconstruct_path(Node TargetNode)
        {
            List<Vector2> shortestPath = new List<Vector2>();
            shortestPath.Add(TargetNode.Pos);

            Node current;
            current = TargetNode;
            while ((current = current.CameFrom) != null)
            {
                shortestPath.Add(current.Pos);
            }
            shortestPath.Reverse();

            return shortestPath;
        }
    }
}