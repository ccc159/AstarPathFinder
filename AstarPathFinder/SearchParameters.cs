using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace AstarPathFinder
{
    public class SearchParameters
    {
        public Point3d[,,] Points { get; set; }
        public Box[,,] Boxes { get; set; }
        public bool[,,] Environment { get; set; }
        public Point3d StartLocation { get; set; }
        public Point3d EndLocation { get; set; }
        public int[] StartNodeID { get; set; }
        public int[] EndNodeID { get; set; }
        public Node StartNode { get; set; }
        public Node EndNode { get; set; }

        public int WorldLength;
        public int WorldWidth;
        public int WorldHeight;
        public double DPI;

        public SearchParameters(BoundingBox iBoundingBox, Point3d StartPoint, Point3d EndPoint, List<Brep> Obstacles, double Resolution)
        {
            StartLocation = StartPoint;
            EndLocation = EndPoint;

            if (iBoundingBox.IsDegenerate(-1) == 0)
            {
                Box box = new Box(iBoundingBox);
                Interval domainX = box.X;
                Interval domainY = box.Y;
                Interval domainZ = box.Z;
                double Length = domainX.Length;
                double Width = domainY.Length;
                double Height = domainZ.Length;
                double Distance = StartPoint.DistanceTo(EndPoint);
                DPI = Distance/Resolution;
                WorldLength = Math.Max(Convert.ToInt32((Length - Length%DPI)/DPI), 4);
                WorldWidth = Math.Max(Convert.ToInt32((Width - Width%DPI)/DPI), 4);
                WorldHeight = Math.Max(Convert.ToInt32((Height - Height%DPI)/DPI), 4);
                List<Interval> SegX = DivideDomain(domainX, WorldLength);
                List<Interval> SegY = DivideDomain(domainY, WorldWidth);
                List<Interval> SegZ = DivideDomain(domainZ, WorldHeight);
                WorldLength += 4;
                WorldWidth += 4;
                WorldHeight += 4;
                Boxes = new Box[WorldLength, WorldWidth, WorldHeight];
                Points = new Point3d[WorldLength, WorldWidth, WorldHeight];
                Environment = new bool[WorldLength, WorldWidth, WorldHeight];
                for (int x = 0; x < WorldLength; x++)
                {
                    for (int y = 0; y < WorldWidth; y++)
                    {
                        for (int z = 0; z < WorldHeight; z++)
                        {
                            Boxes[x, y, z] = new Box(Plane.WorldXY, SegX[x], SegY[y], SegZ[z]);
                            if (Boxes[x, y, z].Contains(StartPoint)) StartNodeID = new int[] { x, y, z };
                            if (Boxes[x, y, z].Contains(EndPoint)) EndNodeID = new int[] { x, y, z };
                        }
                    }
                }
                AdjustBoxes();
                for (int x = 0; x < WorldLength; x++)
                {
                    for (int y = 0; y < WorldWidth; y++)
                    {
                        for (int z = 0; z < WorldHeight; z++)
                        {
                            Points[x, y, z] = Boxes[x, y, z].Center;
                            if (BoxIntersectWithObstacle(Boxes[x, y, z], Obstacles, DPI / 100))
                                Environment[x, y, z] = false;
                            else Environment[x, y, z] = true;
                        }
                    }
                }

            }
            else
            {
                Box box = new Box(iBoundingBox);
                Interval domainX = box.X;
                Interval domainY = box.Y;
                Interval domainZ = box.Z;
                List<Interval> SegX;
                List<Interval> SegY;
                List<Interval> SegZ;
                double Length = domainX.Length;
                double Width = domainY.Length;
                double Height = domainZ.Length;
                double Distance = StartPoint.DistanceTo(EndPoint);
                DPI = Distance / Resolution;
                if (Math.Abs(Length) < 0.0001)
                {
                    WorldLength = 1;
                    WorldWidth = Math.Max(Convert.ToInt32((Width - Width % DPI) / DPI), 4);
                    WorldHeight = Math.Max(Convert.ToInt32((Height - Height % DPI) / DPI), 4);
                    SegX = new List<Interval> {domainX};
                    SegY = DivideDomain(domainY, WorldWidth);
                    SegZ = DivideDomain(domainZ, WorldHeight);
                    WorldWidth += 4;
                    WorldHeight += 4;
                }
                else if(Math.Abs(Width) < 0.0001)
                {
                    WorldLength = Math.Max(Convert.ToInt32((Length - Length % DPI) / DPI), 4);
                    WorldWidth = 1;
                    WorldHeight = Math.Max(Convert.ToInt32((Height - Height % DPI) / DPI), 4);
                    SegX = DivideDomain(domainX, WorldLength);
                    SegY = new List<Interval> { domainY };
                    SegZ = DivideDomain(domainZ, WorldHeight);
                    WorldLength += 4;
                    WorldHeight += 4;
                }
                else if (Math.Abs(Height) < 0.0001)
                {
                    WorldLength = Math.Max(Convert.ToInt32((Length - Length % DPI) / DPI), 4);
                    WorldWidth = Math.Max(Convert.ToInt32((Width - Width % DPI) / DPI), 4);
                    WorldHeight = 1;
                    SegX = DivideDomain(domainX, WorldLength);
                    SegY = DivideDomain(domainY, WorldWidth);
                    SegZ = new List<Interval> { domainZ };
                    WorldLength += 4;
                    WorldWidth += 4;
                }
                else
                {
                    WorldLength = Math.Max(Convert.ToInt32((Length - Length % DPI) / DPI), 4);
                    WorldWidth = Math.Max(Convert.ToInt32((Width - Width % DPI) / DPI), 4);
                    WorldHeight = Math.Max(Convert.ToInt32((Height - Height % DPI) / DPI), 4);
                    SegX = DivideDomain(domainX, WorldLength);
                    SegY = DivideDomain(domainY, WorldWidth);
                    SegZ = DivideDomain(domainZ, WorldHeight);
                    WorldLength += 4;
                    WorldWidth += 4;
                    WorldHeight += 4;
                }
                
                Boxes = new Box[WorldLength, WorldWidth, WorldHeight];
                Points = new Point3d[WorldLength, WorldWidth, WorldHeight];
                Environment = new bool[WorldLength, WorldWidth, WorldHeight];
                for (int x = 0; x < WorldLength; x++)
                {
                    for (int y = 0; y < WorldWidth; y++)
                    {
                        for (int z = 0; z < WorldHeight; z++)
                        {
                            Boxes[x, y, z] = new Box(Plane.WorldXY, SegX[x], SegY[y], SegZ[z]);
                            if (Boxes[x, y, z].Contains(StartPoint)) StartNodeID = new int[] { x, y, z };
                            if (Boxes[x, y, z].Contains(EndPoint)) EndNodeID = new int[] { x, y, z };
                        }
                    }
                }
                AdjustBoxes();
                for (int x = 0; x < WorldLength; x++)
                {
                    for (int y = 0; y < WorldWidth; y++)
                    {
                        for (int z = 0; z < WorldHeight; z++)
                        {
                            Points[x, y, z] = Boxes[x, y, z].Center;
                            if (BoxIntersectWithObstacle(Boxes[x, y, z], Obstacles, DPI / 100))
                                Environment[x, y, z] = false;
                            else Environment[x, y, z] = true;
                        }
                    }
                }
            }
        }
       
        public SearchParameters(int x, int y, int z, int[] startNodeID, int[] endNodeID, Point3d[,,] points, bool[,,] environment)
        {
            StartNodeID = startNodeID;
            EndNodeID = endNodeID;
            Points = points;
            Environment = environment;
            WorldLength = x;
            WorldWidth = y;
            WorldHeight = z;
            StartLocation = Points[startNodeID[0], startNodeID[1], startNodeID[2]];
            EndLocation = Points[endNodeID[0], endNodeID[1], endNodeID[2]];
        }

        private void AdjustBoxes()
        {
            Vector3d moveToStartPoint = this.StartLocation -this.Boxes[this.StartNodeID[0], this.StartNodeID[1], this.StartNodeID[2]].Center;
            for (int x = 0; x < WorldLength; x++)
            {
                for (int y = 0; y < WorldWidth; y++)
                {
                    for (int z = 0; z < WorldHeight; z++)
                    {
                        Boxes[x, y, z].Transform(Transform.Translation(moveToStartPoint));
                    }
                }
            }
            //foreach (var abox in Boxes)
            //{
            //    abox.Transform(Transform.Translation(moveToStartPoint));
            //}
            Point3d ClosestToEndPoint = this.Boxes[this.EndNodeID[0], this.EndNodeID[1], this.EndNodeID[2]].Center;
            double scaleX;
            double scaleY;
            double scaleZ;
            if (Math.Abs((this.StartLocation.X - ClosestToEndPoint.X)) < 0.0001)
            {
                scaleX = 1;
            }
            else
            {
                scaleX = Math.Abs(this.StartLocation.X - this.EndLocation.X) / Math.Abs(this.StartLocation.X - ClosestToEndPoint.X);
            }
            if (Math.Abs((this.StartLocation.Y - ClosestToEndPoint.Y)) < 0.0001)
            {
                scaleY = 1;
            }
            else
            {
                scaleY = Math.Abs(this.StartLocation.Y - this.EndLocation.Y) / Math.Abs(this.StartLocation.Y - ClosestToEndPoint.Y);
            }
            if (Math.Abs((this.StartLocation.Z - ClosestToEndPoint.Z)) < 0.0001)
            {
                scaleZ = 1;
            }
            else
            {
                scaleZ = Math.Abs(this.StartLocation.Z - this.EndLocation.Z) / Math.Abs(this.StartLocation.Z - ClosestToEndPoint.Z);
            }
            Plane startPointPlane = Plane.WorldXY;
            startPointPlane.Origin = StartLocation;
            //foreach (var abox in Boxes)
            //{
            //    abox.Transform(Transform.Scale(startPointPlane, scaleX,scaleY,scaleZ));
            //}
            for (int x = 0; x < WorldLength; x++)
            {
                for (int y = 0; y < WorldWidth; y++)
                {
                    for (int z = 0; z < WorldHeight; z++)
                    {
                        Boxes[x, y, z].Transform(Transform.Scale(startPointPlane, scaleX, scaleY, scaleZ));
                    }
                }
            }
        }
        

        private List<Interval> DivideDomain(Interval domain, int Res)
        {
            double t0 = domain.T0;
            double t1 = domain.T1;
            double l = (domain.Length / (double)Res);
            List<Interval> subdomains = new List<Interval>();
            subdomains.Add(new Interval(t0-l*2, t0-l));
            subdomains.Add(new Interval(t0 - l, t0));
            for (int i = 0; i < Res; i++)
            {
                subdomains.Add(new Interval(t0 + i * l, t0 + (i + 1) * l));
            }
            subdomains.Add(new Interval(t1, t1+l));
            subdomains.Add(new Interval(t1+l, t1 + l*2));
            return subdomains;
        }

        private static bool BoxIntersectWithObstacle(Box box, List<Brep> Obstacles, double Tolerence)
        {
            if (Obstacles.Count == 0) return false;
            Brep myBrep = Brep.CreateFromBox(box);
            if (myBrep == null)
            {
                Point3d[] Corners = box.GetCorners();
                Corners = Point3d.CullDuplicates(Corners, 0.0001);
                if (Corners.Length == 4)
                {
                    NurbsSurface mySurface = NurbsSurface.CreateFromCorners(Corners[0], Corners[1], Corners[2], Corners[3]);
                    myBrep = Brep.CreateFromSurface(mySurface);
                }
                if (Corners.Length == 2)
                {
                    Corners[0].X += 0.01;
                    Corners[0].Y += 0.01;
                    Corners[0].Z += 0.01;
                    BoundingBox b = new BoundingBox(Corners[0], Corners[1]);
                    myBrep = b.ToBrep();
                }
            }
            foreach (var obstacle in Obstacles)
            {
                Point3d[] outPoints;
                Curve[] outCurve;
                var intersect = Intersection.BrepBrep(myBrep, obstacle, Tolerence, out outCurve, out outPoints);
                if (intersect == true)
                {
                    if (outCurve.Length > 0 || outPoints.Length > 0) return true;
                    if (obstacle.IsPointInside(box.Center, Tolerence, false)) return true;
                }
            }
            return false;
        }

        private bool RectangleIntersectWithObstacle(Rectangle3d Rec, List<Brep> Obstacles, double Tolerence)
        {
            if (Obstacles.Count == 0) return false;
            foreach (var obstacle in Obstacles)
            {
                Curve myRec = Rec.ToNurbsCurve();
                Point3d[] outPoints;
                Curve[] outCurve;
                var intersect = Intersection.CurveBrep(myRec, obstacle, Tolerence, out outCurve, out outPoints);
                if (intersect == true)
                {
                    if (outCurve.Length > 0 || outPoints.Length > 0) return true;
                    if (obstacle.IsPointInside(Rec.Center, Tolerence, false)) return true;
                }
            }
            return false;
        }

        

    }
}
