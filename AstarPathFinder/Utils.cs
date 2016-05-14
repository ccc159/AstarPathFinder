using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace AstarPathFinder
{
    public class Utils
    {

        public static List<GeometryBase> CastToGeometry(Point3d StartPoint, Point3d EndPoint, List<Brep> Objects)
        {
            List<GeometryBase> Geoms = new List<GeometryBase>();
            Geoms.Add((GeometryBase)new Point(StartPoint));
            Geoms.Add((GeometryBase)new Point(EndPoint));
            if (Objects.Count > 0) Geoms.AddRange(Objects.Cast<GeometryBase>());
            return Geoms;
        }

        public static BoundingBox ComputeBoundingBox(List<GeometryBase> Objects)
        {
            BoundingBox UnionBox = BoundingBox.Unset;
            foreach (var Object in Objects)
            {
                BoundingBox b = Object.GetBoundingBox(false);
                UnionBox.Union(b);
            }
            return UnionBox;
        }

        public static BoundingBox ScaleBoundingBox(BoundingBox b)
        {
            return b;
            //if ((b.IsDegenerate(-1) == 2) || (b.IsDegenerate(-1) == 3)) return b;
            //Point3d Centroid = b.Center;
            //Brep Bbrep = b.ToBrep();
            //Bbrep.Transform(Rhino.Geometry.Transform.Scale(Centroid, 1));
            //return Bbrep.GetBoundingBox(false);
        }

        public static bool HaveDirectPath(Point3d StartPoint, Point3d EndPoint, List<Brep> Obstacles)
        {
            Line directPath = new Line(StartPoint,EndPoint);
            Curve myCurve = new LineCurve(directPath);
            if (Obstacles.Count == 0) return true;
            foreach (var obstacle in Obstacles)
            {
                Point3d[] outPoints;
                Curve[] outCurve;
                var intersect = Intersection.CurveBrep(myCurve, obstacle, 0.001, out outCurve, out outPoints);
                if (intersect == true)
                {
                    if (outCurve.Length > 0 || outPoints.Length > 0) return false;
                }
            }
            return true;
        }

        public static List<Brep> Screenobstacles(Point3d StartPoint, Point3d EndPoint, List<Brep> Obstacles)
        {
            if (Obstacles.Count == 0) return new List<Brep>();
            BoundingBox myBoundingBox = ComputeBoundingBox(CastToGeometry(StartPoint, EndPoint, new List<Brep>()));
            Box mybox = new Box(myBoundingBox);
            List<Brep> screenedObstacles = new List<Brep>();
            foreach (var obstacle in Obstacles)
            {
                Point3d[] outPoints;
                Curve[] outCurve;
                Brep myBrep = Brep.CreateFromBox(mybox);
                if (myBrep == null)
                {
                    Point3d[] Corners = mybox.GetCorners();
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
                var intersect = Intersection.BrepBrep(myBrep, obstacle, 0.001, out outCurve, out outPoints);
                if (intersect == true)
                {
                    if (outCurve.Length > 0 || outPoints.Length > 0) screenedObstacles.Add(obstacle);
                }
            }
            return screenedObstacles;
        }

        public static Point3d GetCenterPt(Point3d[] Points)
        {
            int i = Points.Length;
            Point3d AllPt = Point3d.Unset;
            foreach (var pt in Points)
            {
                AllPt = AllPt + pt;
            }
            return AllPt / i;
        }
    }

    
}
