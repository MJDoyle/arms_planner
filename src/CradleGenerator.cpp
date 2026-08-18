#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Plane.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopTools_ListOfShape.hxx>


#include <BRepAlgoAPI_Cut.hxx>
#include <BOPAlgo_Options.hxx>

#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>


#include "assembler/CradleGenerator.hpp"

#include "assembler/Config.hpp"

#include "assembler/Logger.hpp"

#include "assembler/quickhull/QuickHull.hpp"

#include "assembler/ARMSConfig.hpp"

struct HullMesh
{
  std::vector<gp_Pnt>    vertices;
  std::vector<uint32_t>  indices;  // 3 per triangle, 0-based
};

// Collect 3D points from a TopoDS_Shape
static void CollectPointsFromShape(const TopoDS_Shape& shape,
    std::vector<gp_Pnt>& outPoints,
    double deflection = 0.01)
{
    // Triangulate (if not already)
    BRepMesh_IncrementalMesh mesher(shape, deflection);
    mesher.Perform();

    TopExp_Explorer ex(shape, TopAbs_FACE);
    for (; ex.More(); ex.Next())
    {
        const TopoDS_Face& F = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(F, loc);
        if (tri.IsNull())
        continue;

        const gp_Trsf& T = loc.Transformation();

        const int nbNodes = tri->NbNodes();

        for (int i = 1; i <= nbNodes; ++i)
        {
        // Works on all versions because:
        // - In ≥7.5: Node(i) is an inline wrapper around Nodes()(i)
        // - In ≤7.4: Node(i) is the only API
        gp_Pnt p = tri->Node(i);

        if (!loc.IsIdentity())
        p.Transform(T);

        outPoints.push_back(p);
        }
    }
}

// Build convex hull triangle mesh from OCCT shape
static HullMesh BuildConvexHullMesh(const TopoDS_Shape& shape,
     double deflection = 0.01)
{
    std::vector<gp_Pnt> pts;
    CollectPointsFromShape(shape, pts, deflection);

    HullMesh out;
    if (pts.size() < 4)
    return out; // too few points for a 3D hull

    // Fill QuickHull point cloud
    std::vector<quickhull::Vector3<double>> cloud;
    cloud.reserve(pts.size());
    for (const gp_Pnt& p : pts)
    cloud.emplace_back(p.X(), p.Y(), p.Z());

    quickhull::QuickHull<double> qh;

    // CCW = true; useOriginalIndices = false (we get a compact vertex buffer)
    auto hull = qh.getConvexHull(cloud, /*CCW*/ true, /*useOriginalIndices*/ false);

    auto& vbuf = hull.getVertexBuffer(); // behaves like array of Vector3<double>
    auto& ibuf = hull.getIndexBuffer();  // std::vector<IndexType>

    out.vertices.reserve(vbuf.size());
    for (size_t i = 0; i < vbuf.size(); ++i)
    {
        const auto& v = vbuf[i];
        out.vertices.emplace_back(v.x, v.y, v.z);
    }

    out.indices.assign(ibuf.begin(), ibuf.end());
    return out;
}

static double ComputeScaleAwareTol(const TopoDS_Shape& s, double rel = 1e-6, double absMin = 1e-6, double absMax = 1e-2)
{
  Bnd_Box bb;
  BRepBndLib::Add(s, bb);
  Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
  bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  const double diag = gp_Vec(gp_Pnt(xmin,ymin,zmin), gp_Pnt(xmax,ymax,zmax)).Magnitude();
  double tol = std::max(absMin, rel * diag);
  tol = std::min(tol, absMax);
  return tol;
}

static bool IsDegenerateTri(const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c, double epsLen, double epsArea)
{
  if (a.Distance(b) < epsLen) return true;
  if (b.Distance(c) < epsLen) return true;
  if (c.Distance(a) < epsLen) return true;

  gp_Vec ab(a, b), ac(a, c);
  const double area2 = ab.Crossed(ac).Magnitude(); // = 2*area
  return area2 < (2.0 * epsArea);
}

TopoDS_Shape makeConvexHullSolid_Robust(const std::vector<gp_Pnt>& verts,
                                       const std::vector<uint32_t>& indices,
                                        const double sewTol = 1e-4)
{
  if (verts.empty() || indices.empty() || (indices.size() % 3) != 0)
  {
    std::cerr << "ERROR: Invalid hull mesh\n";
    return TopoDS_Shape();
  }

  // Heuristics in model units (meters in your pipeline, presumably)
  const double epsLen  = 1e-4;   // edge length threshold
  const double epsArea = 1e-8;  // triangle area threshold

  // Use a sewing tolerance that is not insanely tight for numeric hull vertices
  //const double sewTol = 1e-4; // start here; if your models are tiny, reduce

  BRepBuilderAPI_Sewing sewing(sewTol, /*option1*/ Standard_True, /*option2*/ Standard_True, /*option3*/ Standard_True);

  size_t added = 0, skipped = 0;

  for (size_t t = 0; t < indices.size(); t += 3)
  {
    const uint32_t i0 = indices[t];
    const uint32_t i1 = indices[t + 1];
    const uint32_t i2 = indices[t + 2];

    if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size())
    {
      skipped++;
      continue;
    }
    if (i0 == i1 || i1 == i2 || i2 == i0)
    {
      skipped++;
      continue;
    }

    const gp_Pnt& p0 = verts[i0];
    const gp_Pnt& p1 = verts[i1];
    const gp_Pnt& p2 = verts[i2];

    if (IsDegenerateTri(p0, p1, p2, epsLen, epsArea))
    {
      skipped++;
      continue;
    }

    // Build a planar face explicitly on the triangle plane (more robust than MakeFace(wire) guessing)
    gp_Vec n = gp_Vec(p0, p1).Crossed(gp_Vec(p0, p2));
    if (n.Magnitude() < 1e-15)
    {
      skipped++;
      continue;
    }

    Handle(Geom_Plane) plane = new Geom_Plane(p0, gp_Dir(n));

    BRepBuilderAPI_MakePolygon mkPoly(p0, p1, p2, /*close*/ true);
    if (!mkPoly.IsDone())
    {
      skipped++;
      continue;
    }

    BRepBuilderAPI_MakeFace mkFace(plane, mkPoly.Wire(), /*Inside*/ true);
    if (!mkFace.IsDone())
    {
      skipped++;
      continue;
    }

    sewing.Add(mkFace.Face());
    added++;
  }

  if (added == 0)
  {
    std::cerr << "ERROR: All hull triangles were degenerate / failed to build faces\n";
    return TopoDS_Shape();
  }

  sewing.Perform();
  TopoDS_Shape stitched = sewing.SewedShape();
  if (stitched.IsNull())
  {
    std::cerr << "ERROR: Sewing produced null shape\n";
    return TopoDS_Shape();
  }

  // // Extract a shell (handle compound / multiple shells)
  // TopoDS_Shell shell;
  // for (TopExp_Explorer ex(stitched, TopAbs_SHELL); ex.More(); ex.Next())
  // {
  //   shell = TopoDS::Shell(ex.Current());
  //   break;
  // }
  // if (shell.IsNull())
  // {
  //   std::cerr << "ERROR: No shell produced by sewing\n";
  //   return TopoDS_Shape();
  // }

  ShapeUpgrade_UnifySameDomain unify(stitched, true, true, true);
unify.Build();
TopoDS_Shape unified = unify.Shape();

// unified might be a shell, a compound containing shells, or even a solid.
// Extract a shell again:
TopoDS_Shell unifiedShell;
for (TopExp_Explorer ex(unified, TopAbs_SHELL); ex.More(); ex.Next())
{
  unifiedShell = TopoDS::Shell(ex.Current());
  break;
}

if (unifiedShell.IsNull())
{
  // If unify somehow returned a solid already, handle that too:
  if (unified.ShapeType() == TopAbs_SOLID)
  {
    TopoDS_Solid solid = TopoDS::Solid(unified);
    return solid;
  }

  std::cerr << "ERROR: No shell produced after unify\n";
  return TopoDS_Shape();
}


  BRepBuilderAPI_MakeSolid mkSolid(unifiedShell);
  if (!mkSolid.IsDone())
  {
    std::cerr << "ERROR: MakeSolid failed\n";
    return TopoDS_Shape();
  }

  TopoDS_Solid solid = mkSolid.Solid();

  // Ensure proper orientation for a closed solid (important for booleans)
  BRepLib::OrientClosedSolid(solid);

  // ShapeFix pass (still useful)
  Handle(ShapeFix_Shape) fix = new ShapeFix_Shape();
  fix->Init(solid);
  fix->Perform();
  TopoDS_Shape fixed = fix->Shape();

  BRepCheck_Analyzer ana(fixed);
  if (!ana.IsValid())
  {
    std::cerr << "WARNING: Hull solid still not perfectly valid "
              << "(added=" << added << ", skipped=" << skipped << ", sewTol=" << sewTol << ")\n";
  }

  return fixed;
}

TopoDS_Shape safeCut(const TopoDS_Shape& A, const TopoDS_Shape& B, double fuzzy = 1e-4)
{
  if (A.IsNull() || B.IsNull())
  {
    std::cerr << "Cut failed: A or B is null\n";
    return TopoDS_Shape();
  }

  try
  {
    TopTools_ListOfShape args;
    args.Append(A);
    TopTools_ListOfShape tools;
    tools.Append(B);

    BRepAlgoAPI_Cut cut;
    cut.SetArguments(args);
    cut.SetTools(tools);
    cut.SetFuzzyValue(fuzzy);
    cut.Build();

    if (!cut.IsDone())
    {
      std::cerr << "Cut failed: not done\n";
      return TopoDS_Shape();
    }

    TopoDS_Shape res = cut.Shape();
    if (res.IsNull())
    {
      std::cerr << "Cut failed: result is null\n";
      return TopoDS_Shape();
    }

    // (Optional) validity check — don’t treat invalid as fatal, but log it
    BRepCheck_Analyzer anaRes(res);
    if (!anaRes.IsValid())
      std::cerr << "WARNING: Cut result is invalid\n";

    return res;
  }
  catch (Standard_Failure& e)
  {
    std::cerr << "Boolean exception: " << e.GetMessageString() << "\n";
    return TopoDS_Shape();
  }
}
  
float CradleGenerator::createSimpleNegative(float bay_size, int bay_index, const std::string& output_dir)
{
    RCLCPP_INFO(logger(), "Shape: %s", name_.c_str());
  
    gp_Pnt shape_position = ShapeCentroid(shape_);
    TopoDS_Shape jigBlock = BRepPrimAPI_MakeBox(shape_position, bay_size + 0.3f, bay_size + 0.3f, JIG_HEIGHT);
    jigBlock = ShapeSetCentroid(jigBlock, gp_Pnt(shape_position.X(), shape_position.Y(), shape_position.Z()));

    // Round the 4 vertical (Z-axis) external corners with a 5 mm fillet
    {
        BRepFilletAPI_MakeFillet fillet(jigBlock);
        for (TopExp_Explorer ex(jigBlock, TopAbs_EDGE); ex.More(); ex.Next())
        {
            const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
            BRepAdaptor_Curve curve(edge);
            if (curve.GetType() == GeomAbs_Line)
            {
                gp_Dir dir = curve.Line().Direction();
                if (std::abs(dir.Dot(gp_Dir(0, 0, 1))) > 0.99)
                    fillet.Add(5.0, edge);
            }
        }
        fillet.Build();
        if (fillet.IsDone())
            jigBlock = fillet.Shape();
        else
            RCLCPP_WARN(logger(), "Jig block corner fillet failed, using sharp corners");
    }

    // Notch
    TopoDS_Shape notch = BRepPrimAPI_MakeCylinder(3, 4);
    notch = ShapeSetCentroid(notch,
                             gp_Pnt(shape_position.X() - bay_size / 2,
                                   shape_position.Y() - bay_size / 2,
                                   shape_position.Z() + JIG_HEIGHT / 2));
    {
      TopoDS_Shape tmp = safeCut(jigBlock, notch, /*fuzzy*/ 1e-6);
      if (!tmp.IsNull()) jigBlock = tmp;
    }

    RCLCPP_INFO(logger(), "Pre build");
    // Convex hull mesh from triangulated points (deflection affects this!)
    HullMesh hull = BuildConvexHullMesh(shape_, /*deflection*/ 0.01);
    RCLCPP_INFO(logger(), "Built convex hull of %ld vertices and %ld indices",
                hull.vertices.size(), hull.indices.size());
  
    TopoDS_Shape convex_shape;

    double sew_tol = 1e-10;

    while (true)
    {
      RCLCPP_INFO(logger(), "Attempting to make convex scalled shape with sew_tol: %f", sew_tol);

      convex_shape = makeConvexHullSolid_Robust(hull.vertices, hull.indices, sew_tol);

      if (convex_shape.IsNull())
      {
        RCLCPP_ERROR(logger(), "Convex hull B-Rep build failed (null). Aborting.");
        return 0.0f;
      }

      BRepCheck_Analyzer ana(convex_shape);
      if (!ana.IsValid())
      {
        RCLCPP_WARN(logger(), "Not valid");


        sew_tol *= 10;
      }

      else
      {
        break;
      }
    } 

    RCLCPP_INFO(logger(), "Made convex");

    // Scale
    const double largest_shape_axis = std::max(ShapeAxisSize(convex_shape, 0), ShapeAxisSize(convex_shape, 1));
    const float scaling_distance = scaling_distance_;
    const float scaling_factor = float((largest_shape_axis + scaling_distance) / largest_shape_axis);
  
    TopoDS_Shape scaled_convex_shape = UniformScaleShape(convex_shape, scaling_factor);
    RCLCPP_INFO(logger(), "Scaled shape");

    // (Optional) one more fix pass
    
    Handle(ShapeFix_Shape) fix = new ShapeFix_Shape();
    fix->Init(scaled_convex_shape);
    fix->Perform();
    scaled_convex_shape = fix->Shape();

    RCLCPP_INFO(logger(), "Fixed convex scaled shape");

    BRepCheck_Analyzer ana(scaled_convex_shape);
    if (!ana.IsValid())
    {
      RCLCPP_WARN(logger(), "Shape not valid");
    }

    // Debug exports (safe)
    SaveShapeAsSTL(scaled_convex_shape, WORKING_DIR + name_ + "_convex_scaled_shape.stl");
    SaveShapeAsSTL(shape_,       WORKING_DIR + name_ + "_shape.stl");
  
    TopoDS_Shape final_jig = jigBlock;
    float jig_part_z_offset = 0.0f;

    // Pre-compute shape centroid z — used as the shallowest-viable-jig stopping criterion.
    // Once the jig top reaches the part's centre of mass, it captures enough geometry for
    // lateral stability; stepping higher only makes the jig unnecessarily deep.
    const double shape_centroid_z = ShapeCentroid(shape_).Z();

    int i = -1;
    for (float z = ShapeLowestPoint(shape_) - (JIG_HEIGHT * 0.5f + 1.0f);
         z < ShapeLowestPoint(shape_) + JIG_HEIGHT * 0.5f - 1.0f;
         z += 1.0f)
    {
      i++;
  
      TopoDS_Shape movedBlock = ShapeSetCentroid(jigBlock, gp_Pnt(shape_position.X(), shape_position.Y(), z));
  

      SaveShapeAsSTL(makeCompound({movedBlock, scaled_convex_shape}), WORKING_DIR + name_ + "_jig_part_precut_" + std::to_string(i) + ".stl");

      // Cut
      TopoDS_Shape jig = safeCut(movedBlock, scaled_convex_shape, /*fuzzy*/ 1e-5);
      if (jig.IsNull())
      {
        RCLCPP_WARN(logger(), "Cut failed at step %d (z=%.3f). Skipping this step.", i, z);
        continue; // or break; depending on what you want
      }
  
      RCLCPP_INFO(logger(), "Performed cut (step %d)", i);
      SaveShapeAsSTL(jig, WORKING_DIR + name_ + "_partial_jig_" + std::to_string(i) + ".stl");
  
      int num_downwards_faces = 0;
  
      // IMPORTANT: outwardFaceNormal(face) can throw Standard_Failure on bad faces
      for (TopExp_Explorer exp(jig, TopAbs_FACE); exp.More(); exp.Next())
      {
        const TopoDS_Face face = TopoDS::Face(exp.Current());
        if (face.IsNull())
          continue;
  
        try
        {
          // Ignore non-planar
          Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
          if (surf.IsNull() || !surf->IsKind(STANDARD_TYPE(Geom_Plane)))
            continue;
  
          //TODO ignore faces with tiny area

          gp_Dir normal = outwardFaceNormal(face);
  
          // Ignore faces that don't point downwards
          if (normal.Angle(UPWARDS) <= M_PI * 0.6)
            continue;

          RCLCPP_INFO(logger(), "Downward face with angle %f and area %f and centroid %f %f %f", normal.Angle(UPWARDS), faceArea(face), ShapeCentroid(face).X(), ShapeCentroid(face).Y(), ShapeCentroid(face).Z());
  
          num_downwards_faces++;
        }
        catch (Standard_Failure& e)
        {
          // Don’t die because a single face is awkward
          std::cerr << "WARNING: normal computation failed: " << e.GetMessageString() << "\n";
          continue;
        }
      }
  
      if (num_downwards_faces > 1)
      {
        RCLCPP_INFO(logger(), "%d downwards faces detected)", num_downwards_faces);
        break;
      }
  
      jig_part_z_offset = float(ShapeCentroid(shape_).Z() - ShapeCentroid(jig).Z());
      final_jig = jig;

      // Primary early-stop: once the jig top reaches the part's centre of mass the jig
      // already supports everything below it — no need to go deeper.
      {
        const Standard_Real jig_top_centroid_check = ShapeHighestPoint(jig);
        if (jig_top_centroid_check >= shape_centroid_z)
        {
          RCLCPP_INFO(logger(), "Jig top (%.3f) reached part centroid (%.3f) — shallowest viable jig found at step %d",
                      jig_top_centroid_check, shape_centroid_z, i);
          break;
        }
      }

      // Stop if no downward-facing faces of the part remain above the current jig top.
      // There's no geometry left to capture by stepping higher.
      const double downward_normal_threshold = 0.5; // min |z| for a face to count as downward-facing
      Standard_Real jig_top = ShapeHighestPoint(jig);
      bool any_downward_face_above_jig = false;
      for (TopExp_Explorer fexp(shape_, TopAbs_FACE); fexp.More(); fexp.Next())
      {
        const TopoDS_Face face = TopoDS::Face(fexp.Current());
        if (face.IsNull())
          continue;
        try
        {
          Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
          if (surf.IsNull() || !surf->IsKind(STANDARD_TYPE(Geom_Plane)))
            continue;
          gp_Dir normal = outwardFaceNormal(face);
          if (normal.Z() > -downward_normal_threshold)
            continue;
          if (ShapeCentroid(face).Z() > jig_top)
          {
            any_downward_face_above_jig = true;
            break;
          }
        }
        catch (Standard_Failure&)
        {
          continue;
        }
      }
      if (!any_downward_face_above_jig)
      {
        RCLCPP_INFO(logger(), "No downward-facing part faces above jig top (%.3f mm), stopping early", jig_top);
        break;
      }
    }
  
    if (final_jig.IsNull())
    {
      RCLCPP_ERROR(logger(), "Final jig is null. Aborting STL export.");
      return 0.0f;
    }

    // Translate the final jig so its centroid z = JIG_CENTER_Z.
    // This ensures every jig STL is saved at the same known z in world space,
    // making the Foxglove marker position consistent (all at z=JIG_CENTER_Z).
    // jig_part_z_offset = part_z - jig_z_before_shift, so
    //   part_bay_z = JIG_CENTER_Z + jig_part_z_offset  (computed in generateNegatives)
    // correctly places the part at the same relative height above the jig.
    {
      gp_Pnt jig_centroid = ShapeCentroid(final_jig);
      final_jig = ShapeSetCentroid(
          final_jig,
          gp_Pnt(jig_centroid.X(), jig_centroid.Y(), JIG_CENTER_Z));
    }

    // Mesh + export final
    BRepMesh_IncrementalMesh mesher(final_jig, 0.01);
    mesher.Perform();
  
    StlAPI_Writer writer;
    std::stringstream ss;
    ss << output_dir << "jig_" << name_ << "_size_" << bay_size << "_index_" << bay_index << ".stl";
    writer.Write(final_jig, ss.str().c_str());
  
    return jig_part_z_offset;
}
  



