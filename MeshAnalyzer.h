#ifndef MESHANALYZER_H 
#define MESHANALYZER_H 
#include <RecFusion.h> 

class MeshAnalyzer {

public:
    bool needsCleaning(const RecFusion::Mesh& mesh, double minArea, double maxArea);
    bool needsSmoothing(const RecFusion::Mesh& mesh, double angleThreshold);
    bool needsDecimation(const RecFusion::Mesh& mesh, double ratioThreshold);
    bool needsHoleFilling(const RecFusion::Mesh& mesh, double ratioThreshold);

private:
    double calculateAverageEdgeLength(const RecFusion::Mesh& mesh);
    double calculateAverageDihedralAngle(const RecFusion::Mesh& mesh);
};
#endif  
