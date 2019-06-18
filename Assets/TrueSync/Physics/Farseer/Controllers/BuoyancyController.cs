using System.Collections.Generic;

namespace TrueSync.Physics2D
{
    public sealed class BuoyancyController : Controller
    {
        /// <summary>
        /// Controls the rotational drag that the fluid exerts on the bodies within it. Use higher values will simulate thick fluid, like honey, lower values to
        /// simulate water-like fluids. 
        /// </summary>
        public FP AngularDragCoefficient;

        /// <summary>
        /// Density of the fluid. Higher values will make things more buoyant, lower values will cause things to sink.
        /// </summary>
        public FP Density;

        /// <summary>
        /// Controls the linear drag that the fluid exerts on the bodies within it.  Use higher values will simulate thick fluid, like honey, lower values to
        /// simulate water-like fluids.
        /// </summary>
        public FP LinearDragCoefficient;

        /// <summary>
        /// Acts like waterflow. Defaults to 0,0.
        /// </summary>
        public TSVector2 Velocity;

        private AABB _container;

        private TSVector2 _gravity;
        private TSVector2 _normal;
        private FP _offset;
        private Dictionary<int, Body> _uniqueBodies = new Dictionary<int, Body>();

        /// <summary>
        /// Initializes a new instance of the <see cref="BuoyancyController"/> class.
        /// </summary>
        /// <param name="container">Only bodies inside this AABB will be influenced by the controller</param>
        /// <param name="density">Density of the fluid</param>
        /// <param name="linearDragCoefficient">Linear drag coefficient of the fluid</param>
        /// <param name="rotationalDragCoefficient">Rotational drag coefficient of the fluid</param>
        /// <param name="gravity">The direction gravity acts. Buoyancy force will act in opposite direction of gravity.</param>
        public BuoyancyController(AABB container, FP density, FP linearDragCoefficient, FP rotationalDragCoefficient, TSVector2 gravity)
            : base(ControllerType.BuoyancyController)
        {
            Container = container;
            _normal = new TSVector2(0, 1);
            Density = density;
            LinearDragCoefficient = linearDragCoefficient;
            AngularDragCoefficient = rotationalDragCoefficient;
            _gravity = gravity;
        }

        public AABB Container
        {
            get { return _container; }
            set
            {
                _container = value;
                _offset = _container.UpperBound.y;
            }
        }

        public override void Update(FP dt)
        {
            _uniqueBodies.Clear();
            World.QueryAABB(fixture =>
                                {
                                    if (fixture.Body.IsStatic || !fixture.Body.Awake)
                                        return true;

                                    if (!_uniqueBodies.ContainsKey(fixture.Body.BodyId))
                                        _uniqueBodies.Add(fixture.Body.BodyId, fixture.Body);

                                    return true;
                                }, ref _container);

            foreach (KeyValuePair<int, Body> kv in _uniqueBodies)
            {
                Body body = kv.Value;

                TSVector2 areac = TSVector2.zero;
                TSVector2 massc = TSVector2.zero;
                FP area = 0;
                FP mass = 0;

                for (int j = 0; j < body.FixtureList.Count; j++)
                {
                    Fixture fixture = body.FixtureList[j];

                    if (fixture.Shape.ShapeType != ShapeType.Polygon && fixture.Shape.ShapeType != ShapeType.Circle)
                        continue;

                    Shape shape = fixture.Shape;

                    TSVector2 sc;
                    FP sarea = shape.ComputeSubmergedArea(ref _normal, _offset, ref body._xf, out sc);
                    area += sarea;
                    areac.x += sarea * sc.x;
                    areac.y += sarea * sc.y;

                    mass += sarea * shape.Density;
                    massc.x += sarea * sc.x * shape.Density;
                    massc.y += sarea * sc.y * shape.Density;
                }

                areac.x /= area;
                areac.y /= area;
                massc.x /= mass;
                massc.y /= mass;

                if (area < Settings.Epsilon)
                    continue;

                //Buoyancy
                TSVector2 buoyancyForce = -Density * area * _gravity;
                body.ApplyForce(buoyancyForce, massc);

                //Linear drag
                TSVector2 dragForce = body.GetLinearVelocityFromWorldPoint(areac) - Velocity;
                dragForce *= -LinearDragCoefficient * area;
                body.ApplyForce(dragForce, areac);

                //Angular drag
                body.ApplyTorque(-body.Inertia / body.Mass * area * body.AngularVelocity * AngularDragCoefficient);
            }
        }
    }
}