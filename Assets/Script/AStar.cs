using UnityEngine;
using System.Collections;
using System.Collections.Generic;


namespace AStar
{
    public struct Point
    {
        public int X;
        public int Y;

        public Point(int x, int y)
        {
            X = x;
            Y = y;
        }

        public static bool operator ==(Point p1, Point p2)
        {
            return p1.X == p2.X && p1.Y == p2.Y;
        }
        public static bool operator !=(Point p1, Point p2)
        {
            return p1.X != p2.X && p1.Y != p2.Y;
        }
        public static Point operator +(Point p1, Point p2)
        {
            return new Point(p1.X + p2.X, p1.Y + p2.Y);
        }
        public static Point operator -(Point p1, Point p2)
        {
            return new Point(p1.X - p2.X, p1.Y - p2.Y);
        }

        public override string ToString()
        {
            return X.ToString() + "_" + Y.ToString();
        }
    }

    public struct Rectangle
    {
        public int X;
        public int Y;
        public uint SizeX;
        public uint SizeY;

        public Rectangle(int x, int y, uint sx, uint sy)
        {
            X = x;
            Y = y;
            SizeX = sx;
            SizeY = sy;
        }
        public bool Contains(Point p)
        {
            return p.X >= X && p.Y >= Y && p.X < X + SizeX && p.Y < Y + SizeY;
        }
    }

    public enum CompassDirections
    {
        NotSet = 0,
        North = 1,
        East = 2,
        South = 3,
        West = 4,
    }
    
    // ICostGetter 获取从当前节点向某个方向移动时的代价。
    public interface ICostGetter
    {
        int GetCost(Point currentNodeLoaction, CompassDirections moveDirection);
    }
    // SimpleCostGetter ICostGetter接口的简化实现
    public class SimpleCostGetter : ICostGetter
    {
        public int GetCost(Point currentNodeLoaction, CompassDirections moveDirection)
        {
            if (moveDirection == CompassDirections.NotSet)
            {
                return 0;
            }
            return 1;
        }
    }

    // AStarNode 用于保存规划到当前节点时的各个Cost值以及父节点。
    public class AStarNode
    {
        public AStarNode(Point loc, AStarNode previous, int _costG, int _costH)
        {
            this.location = loc;
            this.previousNode = previous;
            this.costG = _costG;
            this.costH = _costH;
        }

        private Point location = new Point(0, 0);
        public Point Location
        {
            get { return location; }
        }

        private AStarNode previousNode = null;
        public AStarNode PreviousNode
        {
            get { return previousNode; }
        }

        // CostF 从起点导航经过本节点然后再到目的节点的估算总代价。
        public int CostF
        {
            get
            {
                return this.costG + this.costH;
            }
        }

        // CostG 从起点导航到本节点的代价。
        private int costG = 0;
        public int CostG
        {
            get { return costG; }
        }

        // CostH 使用启发式方法估算的从本节点到目的节点的代价。
        private int costH = 0;
        public int CostH
        {
            get { return costH; }
        }

        // ResetPreviousNode 当从起点到达本节点有更优的路径时，调用该方法采用更优的路径。
        public void ResetPreviousNode(AStarNode previous, int _costG)
        {
            this.previousNode = previous;
            this.costG = _costG;
        }

        public override string ToString()
        {
            return this.location.ToString();
        }

    }

    // RoutePlanData 用于封装一次路径规划过程中的规划信息。
    public class RoutePlanData
    {
        // CellMap 地图的矩形大小。经过单元格标准处理。
        private Rectangle cellMap;
        public Rectangle CellMap
        {
            get { return cellMap; }
        }

        // ClosedList 关闭列表，即存放已经遍历处理过的节点。
        private IList<AStarNode> closedList = new List<AStarNode>();
        public IList<AStarNode> ClosedList
        {
            get { return closedList; }
        }

        // OpenedList 开放列表，即存放已经开发但是还未处理的节点。
        private IList<AStarNode> openedList = new List<AStarNode>();
        public IList<AStarNode> OpenedList
        {
            get { return openedList; }
        }

        // Destination 目的节点的位置。
        private Point destination;
        public Point Destination
        {
            get { return destination; }
        }
        
        public RoutePlanData(Rectangle map, Point _destination)
        {
            this.cellMap = map;
            this.destination = _destination;
        }
    }


    /// <summary>
    /// AStarRoutePlanner A*路径规划。每个单元格Cell的位置用Point表示
    /// F = G + H 。
    /// G = 从起点A，沿着产生的路径，移动到网格上指定方格的移动耗费。
    /// H = 从网格上那个方格移动到终点B的预估移动耗费。使用曼哈顿方法，它计算从当前格到目的格之间水平和垂直的方格的数量总和，忽略对角线方向。
    /// </summary>
    public class AStarRoutePlanner
    {
        static CompassDirections[] Directions = new CompassDirections[] {CompassDirections.East, CompassDirections.South, CompassDirections.West, CompassDirections.North};
        static Point[] DirModifies = new Point[]{new Point(1, 0), new Point(0, 1), new Point(-1, 0), new Point(0, -1)};

        private uint lineCount = 10;   //反映地图高度，对应Y坐标
        private uint columnCount = 10; //反映地图宽度，对应X坐标

        private ICostGetter costGetter = new SimpleCostGetter();

        private bool[][] obstacles = null; //障碍物位置，维度：Column * Line         
        public bool[][] Obstacles
        {
            get { return obstacles; }
        }

        public AStarRoutePlanner() : this(10, 10, new SimpleCostGetter())
        {}

        public AStarRoutePlanner(uint _lineCount, uint _columnCount, ICostGetter _costGetter)
        {
            this.lineCount = _lineCount;
            this.columnCount = _columnCount;
            this.costGetter = _costGetter;

            this.InitializeObstacles();
        }

        // InitializeObstacles 将所有位置均标记为无障碍物。
        private void InitializeObstacles()
        {
            this.obstacles = new bool[this.columnCount][];
            for (int i = 0; i < this.columnCount; i++)
            {
                this.obstacles[i] = new bool[this.lineCount];
            }

            for (int i = 0; i < this.columnCount; i++)
            {
                for (int j = 0; j < this.lineCount; j++)
                {
                    this.obstacles[i][j] = false;
                }
            }
        }
        
        // Initialize 在路径规划之前先设置障碍物位置。
        public void Initialize(IList<Point> obstaclePoints)
        {
            if (obstacles != null)
            {
                foreach (Point pt in obstaclePoints)
                {
                    this.obstacles[pt.X][pt.Y] = true;
                }
            }
        }


        public IList<Point> Plan(Point start, Point destination)
        {
            Rectangle map = new Rectangle(0, 0, this.columnCount, this.lineCount);

            if ((!map.Contains(start)) || (!map.Contains(destination)))
            {
                throw new System.Exception("StartPoint or Destination not in the current map!");
            }
            
            AStarNode startNode = new AStarNode(start, null, 0, 0);
            RoutePlanData routePlanData = new RoutePlanData(map, destination);
            routePlanData.OpenedList.Add(startNode);

            AStarNode currenNode = startNode;

            //从起始节点开始进行递归调用
            return DoPlan(routePlanData, currenNode);

        }
        
        private IList<Point> DoPlan(RoutePlanData routePlanData, AStarNode currenNode)
        {
            for (int i = 0; i < Directions.Length; ++ i)
            {
                CompassDirections direction = Directions[i];
                Point nextCell = currenNode.Location + DirModifies[i];// GeometryHelper.GetAdjacentPoint(currenNode.Location, direction);

                if (!routePlanData.CellMap.Contains(nextCell)) //相邻点已经在地图之外
                {
                    continue;
                }

                if (this.obstacles[nextCell.X][nextCell.Y]) //下一个Cell为障碍物
                {
                    continue;
                }
                
                int costG = this.costGetter.GetCost(currenNode.Location, direction);
                int costH = Mathf.Abs(nextCell.X - routePlanData.Destination.X) + Mathf.Abs(nextCell.Y - routePlanData.Destination.Y);

                if (costH == 0) //costH为0，表示相邻点就是目的点，规划完成，构造结果路径
                {
                    IList<Point> route = new List<Point>();
                    route.Add(routePlanData.Destination);
                    route.Insert(0, currenNode.Location);

                    AStarNode tempNode = currenNode;
                    while (tempNode.PreviousNode != null)
                    {
                        route.Insert(0, tempNode.PreviousNode.Location);
                        tempNode = tempNode.PreviousNode;
                    }
                    return route;
                }

                AStarNode existNode = this.GetNodeOnLocation(nextCell, routePlanData);
                if (existNode != null)
                {
                    if (existNode.CostG > costG)
                    {
                        //如果新的路径代价更小，则更新该位置上的节点的原始路径
                        existNode.ResetPreviousNode(currenNode, costG);
                    }
                }
                else
                {
                    AStarNode newNode = new AStarNode(nextCell, currenNode, costG, costH);
                    routePlanData.OpenedList.Add(newNode);
                }

            }
            
            //将已遍历过的节点从开放列表转移到关闭列表
            routePlanData.OpenedList.Remove(currenNode);
            routePlanData.ClosedList.Add(currenNode);

            AStarNode minCostNode = this.GetMinCostNode(routePlanData.OpenedList);
            if (minCostNode == null) //表明从起点到终点之间没有任何通路。
            {
                return null;
            }

            //对开放列表中的下一个代价最小的节点作递归调用
            return this.DoPlan(routePlanData, minCostNode);
        }

        // GetNodeOnLocation 目标位置location是否已存在于开放列表或关闭列表中
        private AStarNode GetNodeOnLocation(Point location, RoutePlanData routePlanData)
        {
            foreach (AStarNode temp in routePlanData.OpenedList)
            {
                if (temp.Location == location)
                {
                    return temp;
                }
            }
            foreach (AStarNode temp in routePlanData.ClosedList)
            {
                if (temp.Location == location)
                {
                    return temp;
                }
            }
            return null;
        }
        
        // GetMinCostNode 从开放列表中获取代价F最小的节点，以启动下一次递归
        private AStarNode GetMinCostNode(IList<AStarNode> openedList)
        {
            if (openedList.Count == 0)
            {
                return null;
            }
            
            AStarNode target = openedList[0];
            foreach (AStarNode temp in openedList)
            {
                if (temp.CostF < target.CostF)
                {
                    target = temp;
                }
            }
            return target;
        }
    }
}
