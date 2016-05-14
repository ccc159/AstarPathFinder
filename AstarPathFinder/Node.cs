using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;

namespace AstarPathFinder
{
    public enum NodeState { Untested, Open, Closed }
    public class Node
    {
        private Node parentNode;
        public Point3d Location { get; private set; }
        public int[] ID { get; set; }
        public bool IsWalkable { get; set; }
        public double G { get; private set; }
        public double H { get; private set; }
        public NodeState State { get; set; }
        public double F;

        public Node ParentNode
        {
            get { return parentNode; }
            set
            {
                parentNode = value;
                G = parentNode.G + GetTraversalCost(this.Location, this.ParentNode.Location);
                //G = parentNode.G + Location.DistanceTo(parentNode.Location);
            }
        }

        public Node(Point3d location, int[] id, bool isWalkable, Point3d endLocation)
        {
            Location = location;
            ID = id;
            State = NodeState.Untested;
            IsWalkable = isWalkable;
            H = GetTraversalCost(this.Location, endLocation);
            G = 0;
            F = G + H;
        }

        internal static double GetTraversalCost(Point3d location, Point3d otherLocation)
        {
            double intervalX = Math.Abs(location.X - otherLocation.X);
            double intervalY = Math.Abs(location.Y - otherLocation.Y);
            double intervalZ = Math.Abs(location.Z - otherLocation.Z);

            //return (intervalX + intervalY + intervalZ) * 7 ;
            return location.DistanceTo(otherLocation);
        }
    }

}
