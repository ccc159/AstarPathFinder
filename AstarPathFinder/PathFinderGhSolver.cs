using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace AstarPathFinder
{
    public class PathFinderGhSolver
    {
        public List<Brep> Obstacles { get; set; }
        public List<Brep> ScreenedObstacles { get; set; }
        public Point3d StartPoint { get; set; }
        public Point3d EndPoint { get; set; }
        public double Resolution { get; set; }
        public bool DirectPath { get; set; }
        public double Distance { get; set; }
        public BoundingBox TheBoundingBox { get; set; }
        public SearchParameters TheSearchParameters { get; set; }
        public List<Point3d> Path  { get; set; }
        public Polyline Curve { get; set; }
        public int BoundingBoxCondition { get; set; }
        public bool Running { get; set; }
        public string Message { get; set; }
        public double Tolerence { get; set; }

        public PathFinderGhSolver(Point3d startPoint, Point3d endPoint, List<Brep> obstacles,double resolution, double tolerence )
        {
            this.StartPoint = startPoint;
            this.EndPoint = endPoint;
            this.Obstacles = obstacles;
            this.Resolution = resolution;
            this.DirectPath = true;
            this.Distance = StartPoint.DistanceTo(EndPoint);
            this.Tolerence = tolerence;
            this.DirectPath = Utils.HaveDirectPath(StartPoint, EndPoint, Obstacles);
            
        }

        public void Run()
        {
            if (Distance < Tolerence)
            {
                Running = false;
                Message = "You don't need me if the StartPoint and TargetPoint are identical.\n Try with something else :)";
                return;
            }
            if (DirectPath)
            {
                Path = new List<Point3d> { StartPoint, EndPoint };
                Curve = new Polyline(Path);
                Running = true;
                return;
            }
            this.ScreenedObstacles = Utils.Screenobstacles(StartPoint, EndPoint, Obstacles);
            this.TheBoundingBox = Utils.ComputeBoundingBox(Utils.CastToGeometry(StartPoint, EndPoint, ScreenedObstacles));
            this.BoundingBoxCondition = TheBoundingBox.IsDegenerate(-1);
            if (this.BoundingBoxCondition == 2)
            {
                Path = new List<Point3d> { StartPoint, EndPoint };
                Curve = new Polyline(Path);
                Running = true;
                return;
            }
            else if (this.BoundingBoxCondition == 0 || this.BoundingBoxCondition == 1)
            {
                this.TheSearchParameters = new SearchParameters(TheBoundingBox, StartPoint, EndPoint, ScreenedObstacles, Resolution);
                PathFinder iPathFinder = new PathFinder(TheSearchParameters);
                Path = new List<Point3d>();
                Path = iPathFinder.FindPath();
                if (Path.Count == 0)
                {
                    Running = false;
                    Message = "Something went wrong. Could not generate path. Check if start or end point inside obstacles.";
                    return;
                }
                Path.Add(EndPoint);
                Path.Insert(0, StartPoint);
                Curve = new Polyline(Path);
                Running = true;
            }
        } 
    }
}
