using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using Rhino.Geometry;

namespace AstarPathFinder
{
    public class PathFinder
    {
        private int length;
        private int width;
        private int height;
        private Node[,,] nodes;
        private Node startNode;
        private Node endNode;
        private SearchParameters searchParameters;
        private bool vonNeumann;

        public PathFinder(SearchParameters searchParameters,bool ivonNeumann)
        {
            this.searchParameters = searchParameters;
            InitializeNodes();
            int[] id = this.searchParameters.StartNodeID;
            this.startNode = this.nodes[id[0], id[1], id[2]];
            startNode.State = NodeState.Open;
            id = this.searchParameters.EndNodeID;
            this.endNode = this.nodes[id[0], id[1], id[2]];
            this.vonNeumann = ivonNeumann;
        }


        public List<Point3d> FindPath()
        {
            List<Point3d> path = new List<Point3d>();
            bool success = Search(startNode);
            if (success)
            {
                Node node = endNode;
                while (node.ParentNode != null)
                {
                    path.Add(node.Location);
                    node = node.ParentNode;
                }
                path.Reverse();
            }
            return path;
        }

        private void InitializeNodes()
        {
            if (this.searchParameters.Points == null) return;
            this.length = this.searchParameters.WorldLength;
            this.width = this.searchParameters.WorldWidth;
            this.height = this.searchParameters.WorldHeight;
            this.nodes = new Node[this.length, this.width, this.height];
            for (int z = 0; z < this.height; z++)
            {
                for (int y = 0; y < this.width; y++)
                {
                    for (int x = 0; x < this.length; x++)
                    {
                        int[] id = { x, y, z };
                        this.nodes[x, y, z] = new Node(this.searchParameters.Points[x, y, z], id, this.searchParameters.Environment[x, y, z], this.searchParameters.EndLocation);
                    }
                }
            }


        }


        private bool Search(Node currentNode)
        {
            currentNode.State = NodeState.Closed;
            List<Node> nextNodes = GetAdjacentWalkableNodes(currentNode);
            nextNodes.Sort((node1, node2) => node1.F.CompareTo(node2.F));
            foreach (var nextNode in nextNodes)
            {
                if (nextNode.Location == this.endNode.Location)
                {
                    return true;
                }
                else
                {
                    if (Search(nextNode))
                        return true;
                }
            }
            return false;
        }


        private List<Node> GetAdjacentWalkableNodes(Node fromNode)
        {
            List<Node> walkableNodes = new List<Node>();
            List<Node> nextNodes;
            nextNodes = this.vonNeumann ? GetVonNeumannAdjacentNodes(fromNode) : GetMooreAdjacentNodes(fromNode);
            foreach (var node in nextNodes)
            {

                if (!node.IsWalkable) continue;
                if (node.State == NodeState.Closed) continue;
                if (node.State == NodeState.Open)
                {
                    double traversalCost = Node.GetTraversalCost(node.Location, node.ParentNode.Location);
                    double gTemp = fromNode.G + traversalCost;
                    if (gTemp < node.G)
                    {
                        node.ParentNode = fromNode;
                        walkableNodes.Add(node);
                    }
                }
                else
                {
                    node.ParentNode = fromNode;
                    node.State = NodeState.Open;
                    walkableNodes.Add(node);
                }
            }

            return walkableNodes;
        }

        private List<Node> GetVonNeumannAdjacentNodes(Node fromLocation)
        {
            List<Node> locations = new List<Node>();
            var id = fromLocation.ID;
            int x = id[0];
            int y = id[1];
            int z = id[2];
            var points = this.nodes;
            if (z - 1 >= 0) locations.Add(points[x, y, z - 1]);
            if (x - 1 >= 0 && y >= 0) locations.Add(points[x - 1, y , z]);
            if (y - 1 >= 0) locations.Add(points[x , y - 1, z]);
            if (y + 1 < this.width) locations.Add(points[x , y + 1, z]);
            if (x + 1 < this.length) locations.Add(points[x + 1, y, z]);
            if (z + 1 < this.height) locations.Add(points[x + 0, y - 0, z + 1]);
            return locations;
        }

        private List<Node> GetMooreAdjacentNodes(Node fromLocation)
        {
            List<Node> locations = new List<Node>();
            var id = fromLocation.ID;
            int x = id[0];
            int y = id[1];
            int z = id[2];
            var points = this.nodes;
            if (z - 1 >= 0)
            {
                if (x - 1 >= 0 && y - 1 >= 0) locations.Add(points[x - 1, y - 1, z - 1]);
                if (x - 1 >= 0 && y - 0 >= 0) locations.Add(points[x - 1, y + 0, z - 1]);
                if (x - 1 >= 0 && y + 1 < this.width) locations.Add(points[x - 1, y + 1, z - 1]);
                if (y - 1 >= 0) locations.Add(points[x + 0, y - 1, z - 1]);
                locations.Add(points[x + 0, y - 0, z - 1]);
                if (y + 1 < this.width) locations.Add(points[x + 0, y + 1, z - 1]);
                if (x + 1 < this.length && y - 1 >= 0) locations.Add(points[x + 1, y - 1, z - 1]);
                if (x + 1 < this.length) locations.Add(points[x + 1, y - 0, z - 1]);
                if (x + 1 < this.length && y + 1 < this.width) locations.Add(points[x + 1, y + 1, z - 1]);
            }

            if (x - 1 >= 0 && y - 1 >= 0) locations.Add(points[x - 1, y - 1, z]);
            if (x - 1 >= 0 && y - 0 >= 0) locations.Add(points[x - 1, y + 0, z]);
            if (x - 1 >= 0 && y + 1 < this.width) locations.Add(points[x - 1, y + 1, z]);
            if (y - 1 >= 0) locations.Add(points[x + 0, y - 1, z]);
            if (y + 1 < this.width) locations.Add(points[x + 0, y + 1, z]);
            if (x + 1 < this.length && y - 1 >= 0) locations.Add(points[x + 1, y - 1, z]);
            if (x + 1 < this.length) locations.Add(points[x + 1, y - 0, z]);
            if (x + 1 < this.length && y + 1 < this.width) locations.Add(points[x + 1, y + 1, z]);

            if (z + 1 < this.height)
            {
                if (x - 1 >= 0 && y - 1 >= 0) locations.Add(points[x - 1, y - 1, z + 1]);
                if (x - 1 >= 0 && y - 0 >= 0) locations.Add(points[x - 1, y + 0, z + 1]);
                if (x - 1 >= 0 && y + 1 < this.width) locations.Add(points[x - 1, y + 1, z + 1]);
                if (y - 1 >= 0) locations.Add(points[x + 0, y - 1, z + 1]);
                locations.Add(points[x + 0, y - 0, z + 1]);
                if (y + 1 < this.width) locations.Add(points[x + 0, y + 1, z + 1]);
                if (x + 1 < this.length && y - 1 >= 0) locations.Add(points[x + 1, y - 1, z + 1]);
                if (x + 1 < this.length) locations.Add(points[x + 1, y - 0, z + 1]);
                if (x + 1 < this.length && y + 1 < this.width) locations.Add(points[x + 1, y + 1, z + 1]);
            }

            return locations;
        }
    }
}
