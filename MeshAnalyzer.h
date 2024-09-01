#ifndef MESHANALYZER_H
#define MESHANALYZER_H

#include <RecFusion.h>
#include <vector>
#include <map>
class MeshAnalyzer {
public:
    //Sie sind Optional, komm erst nach erfolgreiches PostReko
    /*static bool needsCleaning(const RecFusion::Mesh& mesh, double minArea, double maxArea);
    static bool needsSmoothing(const RecFusion::Mesh& mesh, double angleThreshold);
    static bool needsDecimation(const RecFusion::Mesh& mesh, double ratioThreshold);
    static bool needsHollowing(const RecFusion::Mesh& mesh, double ratioThreshold);
    static bool needsBoundaryFaceRemoval(const RecFusion::Mesh& mesh, double ratioThreshold);

    */
    //Für HoleFilling Funct.
    static bool MeshAnalyzer::needsHoleFilling(const RecFusion::Mesh& mesh, double ratioThreshold, int boundaryEdges, int totalEdges);
    static std::pair<std::map<std::pair<int, int>, int>, std::map<std::pair<int, int>, int>> countBoundaryEdges(const RecFusion::Mesh& mesh);
    //std::pair<int, int>& countBoundaryEdges(const RecFusion::Mesh& mesh);
    static double calculateHoleArea(const RecFusion::Mesh& mesh, std::map<std::pair<int, int>, int> BoundaryEdges);
    //static double calculateHoleArea(const RecFusion::Mesh& mesh, int boundaryEdgesSum);
    static double calculateTriangleArea(const RecFusion::Mesh::Coordinate& v1,
        const RecFusion::Mesh::Coordinate& v2,
        const RecFusion::Mesh::Coordinate& v3);
    static double distance(const RecFusion::Mesh::Coordinate& v1, const RecFusion::Mesh::Coordinate& v2);

private:
    //Sie sind Optional, komm erst nach erfolgreiches PostReko
    /*
    static std::vector<double> calculateComponentAreas(const RecFusion::Mesh& mesh);
    static double calculateAverageDihedralAngle(const RecFusion::Mesh& mesh);
    static RecFusion::Vec3 calculateNormal(const RecFusion::Mesh::Coordinate& v1,
        const RecFusion::Mesh::Coordinate& v2,
        const RecFusion::Mesh::Coordinate& v3);
    static double calculateDihedralAngle(const RecFusion::Vec3& n1, const RecFusion::Vec3& n2);
   
    static double calculateVolume(const RecFusion::Mesh& mesh);
    static int countBoundaryFaces(const RecFusion::Mesh& mesh);
    */
  
};

#endif 