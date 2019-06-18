using System.Collections.Generic;

namespace TrueSync.Physics2D
{
    /// <summary>
    /// Simple class to maintain a terrain. It can keep track
    /// </summary>
    public class Terrain2D
    {
        /// <summary>
        /// World to manage terrain in.
        /// </summary>
        public World World;

        /// <summary>
        /// Center of terrain in world units.
        /// </summary>
        public TSVector2 Center;

        /// <summary>
        /// Width of terrain in world units.
        /// </summary>
        public FP Width;

        /// <summary>
        /// Height of terrain in world units.
        /// </summary>
        public FP Height;

        /// <summary>
        /// Points per each world unit used to define the terrain in the point cloud.
        /// </summary>
        public int PointsPerUnit;

        /// <summary>
        /// Points per cell.
        /// </summary>
        public int CellSize;

        /// <summary>
        /// Points per sub cell.
        /// </summary>
        public int SubCellSize;

        /// <summary>
        /// Number of iterations to perform in the Marching Squares algorithm.
        /// Note: More then 3 has almost no effect on quality.
        /// </summary>
        public int Iterations = 2;

        /// <summary>
        /// Decomposer to use when regenerating terrain. Can be changed on the fly without consequence.
        /// Note: Some decomposerers are unstable.
        /// </summary>
        public TriangulationAlgorithm Decomposer;

        /// <summary>
        /// Point cloud defining the terrain.
        /// </summary>
        private sbyte[,] _terrainMap;

        /// <summary>
        /// Generated bodies.
        /// </summary>
        private List<Body>[,] _bodyMap;

        private FP _localWidth;
        private FP _localHeight;
        private int _xnum;
        private int _ynum;
        private AABB _dirtyArea;
        private TSVector2 _topLeft;

        /// <summary>
        /// Creates a new terrain.
        /// </summary>
        /// <param name="world">The World</param>
        /// <param name="area">The area of the terrain.</param>
        public Terrain2D(World world, AABB area)
        {
            World = world;
            Width = area.Width;
            Height = area.Height;
            Center = area.Center;
        }

        /// <summary>
        /// Creates a new terrain
        /// </summary>
        /// <param name="world">The World</param>
        /// <param name="position">The position (center) of the terrain.</param>
        /// <param name="width">The width of the terrain.</param>
        /// <param name="height">The height of the terrain.</param>
        public Terrain2D(World world, TSVector2 position, FP width, FP height)
        {
            World = world;
            Width = width;
            Height = height;
            Center = position;
        }

        /// <summary>
        /// Initialize the terrain for use.
        /// </summary>
        public void Initialize()
        {
            // find top left of terrain in world space
            _topLeft = new TSVector2(Center.x - (Width * 0.5f), Center.y - (-Height * 0.5f));

            // convert the terrains size to a point cloud size
            _localWidth = Width * PointsPerUnit;
            _localHeight = Height * PointsPerUnit;

            _terrainMap = new sbyte[(int)_localWidth + 1, (int)_localHeight + 1];

            for (int x = 0; x < _localWidth; x++)
            {
                for (int y = 0; y < _localHeight; y++)
                {
                    _terrainMap[x, y] = 1;
                }
            }

            _xnum = (int)(_localWidth / CellSize);
            _ynum = (int)(_localHeight / CellSize);
            _bodyMap = new List<Body>[_xnum, _ynum];

            // make sure to mark the dirty area to an infinitely small box
            _dirtyArea = new AABB(new TSVector2(FP.MaxValue, FP.MaxValue), new TSVector2(FP.MinValue, FP.MinValue));
        }

        /// <summary>
        /// Apply the specified texture data to the terrain.
        /// </summary>
        /// <param name="data"></param>
        /// <param name="offset"></param>
        public void ApplyData(sbyte[,] data, TSVector2 offset = default(TSVector2))
        {
            for (int x = 0; x < data.GetUpperBound(0); x++)
            {
                for (int y = 0; y < data.GetUpperBound(1); y++)
                {
                    if (x + offset.x >= 0 && x + offset.x < _localWidth && y + offset.y >= 0 && y + offset.y < _localHeight)
                    {
                        _terrainMap[(int)(x + offset.x), (int)(y + offset.y)] = data[x, y];
                    }
                }
            }

            RemoveOldData(0, _xnum, 0, _ynum);
        }

        /// <summary>
        /// Modify a single point in the terrain.
        /// </summary>
        /// <param name="location">World location to modify. Automatically clipped.</param>
        /// <param name="value">-1 = inside terrain, 1 = outside terrain</param>
        public void ModifyTerrain(TSVector2 location, sbyte value)
        {
            // find local position
            // make position local to map space
            TSVector2 p = location - _topLeft;

            // find map position for each axis
            p.x = p.x * _localWidth / Width;
            p.y = p.y * -_localHeight / Height;

            if (p.x >= 0 && p.x < _localWidth && p.y >= 0 && p.y < _localHeight)
            {
                _terrainMap[(int)p.x, (int)p.y] = value;

                // expand dirty area
                if (p.x < _dirtyArea.LowerBound.x) _dirtyArea.LowerBound.x = p.x;
                if (p.x > _dirtyArea.UpperBound.x) _dirtyArea.UpperBound.x = p.x;

                if (p.y < _dirtyArea.LowerBound.y) _dirtyArea.LowerBound.y = p.y;
                if (p.y > _dirtyArea.UpperBound.y) _dirtyArea.UpperBound.y = p.y;
            }
        }

        /// <summary>
        /// Regenerate the terrain.
        /// </summary>
        public void RegenerateTerrain()
        {
            //iterate effected cells
            int xStart = (int)(_dirtyArea.LowerBound.x / CellSize);
            if (xStart < 0) xStart = 0;

            int xEnd = (int)(_dirtyArea.UpperBound.x / CellSize) + 1;
            if (xEnd > _xnum) xEnd = _xnum;

            int yStart = (int)(_dirtyArea.LowerBound.y / CellSize);
            if (yStart < 0) yStart = 0;

            int yEnd = (int)(_dirtyArea.UpperBound.y / CellSize) + 1;
            if (yEnd > _ynum) yEnd = _ynum;

            RemoveOldData(xStart, xEnd, yStart, yEnd);

            _dirtyArea = new AABB(new TSVector2(FP.MaxValue, FP.MaxValue), new TSVector2(FP.MinValue, FP.MinValue));
        }

        private void RemoveOldData(int xStart, int xEnd, int yStart, int yEnd)
        {
            for (int x = xStart; x < xEnd; x++)
            {
                for (int y = yStart; y < yEnd; y++)
                {
                    //remove old terrain object at grid cell
                    if (_bodyMap[x, y] != null)
                    {
                        for (int i = 0; i < _bodyMap[x, y].Count; i++)
                        {
                            World.RemoveBody(_bodyMap[x, y][i]);
                        }
                    }

                    _bodyMap[x, y] = null;

                    //generate new one
                    GenerateTerrain(x, y);
                }
            }
        }

        private void GenerateTerrain(int gx, int gy)
        {
            FP ax = gx * CellSize;
            FP ay = gy * CellSize;

            List<Vertices> polys = MarchingSquares.DetectSquares(new AABB(new TSVector2(ax, ay), new TSVector2(ax + CellSize, ay + CellSize)), SubCellSize, SubCellSize, _terrainMap, Iterations, true);
            if (polys.Count == 0) return;

            _bodyMap[gx, gy] = new List<Body>();

            // create the scale vector
            TSVector2 scale = new TSVector2(1f / PointsPerUnit, 1f / -PointsPerUnit);

            // create physics object for this grid cell
            foreach (Vertices item in polys)
            {
                // does this need to be negative?
                item.Scale(ref scale);
                item.Translate(ref _topLeft);
                Vertices simplified = SimplifyTools.CollinearSimplify(item, FP.Zero);

                List<Vertices> decompPolys = Triangulate.ConvexPartition(simplified, Decomposer, true, FP.EN3);

                foreach (Vertices poly in decompPolys)
                {
                    if (poly.Count > 2)
                        _bodyMap[gx, gy].Add(BodyFactory.CreatePolygon(World, poly, 1, null));
                }
            }
        }
    }
}