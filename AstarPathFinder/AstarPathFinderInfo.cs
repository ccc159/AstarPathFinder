using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace AstarPathFinder
{
    public class AstarPathFinderInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "AstarPathFinder";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("3f8d8a4f-e94f-44f0-928e-e57c1d30b1aa");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
