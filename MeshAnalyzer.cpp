#include "MeshAnalyzer.h"
#include <cmath>
#include <vector>
#include <algorithm> // For std::find
#include <iostream>
#include <map>

// Define edge as a pair of vertex indices
struct Edge {
    int vertex1;
    int vertex2;

    Edge(int v1, int v2) {
        if (v1 < v2) {
            vertex1 = v1;
            vertex2 = v2;
        }
        else {
            vertex1 = v2;
            vertex2 = v1;
        }
    }

    // Comparison operator to check if two edges are the same
    bool operator==(const Edge& other) const {
        return (vertex1 == other.vertex1 && vertex2 == other.vertex2);
    }
};

//bool MeshAnalyzer::needsCleaning(const RecFusion::Mesh& mesh, double minArea, double maxArea) {
//    std::vector<double> componentAreas = calculateComponentAreas(mesh);
//    for (double area : componentAreas) {
//        if (area < minArea || area > maxArea) {
//            return true;
//        }
//    }
//    return false;
//}
/*
bool MeshAnalyzer::needsSmoothing(const RecFusion::Mesh& mesh, double angleThreshold) {
    double avgDihedralAngle = calculateAverageDihedralAngle(mesh);
    return avgDihedralAngle < angleThreshold;
}

static double calculateAverageCurvature(const RecFusion::Mesh& mesh) {
    std::vector<double> vertexCurvatures(mesh.vertexCount(), 0.0);

    for (int i = 0; i < mesh.triangleCount(); ++i) {
        RecFusion::Mesh::Triangle tri = mesh.triangle(i);
        RecFusion::Mesh::Coordinate v1 = mesh.vertex(tri.i1);
        RecFusion::Mesh::Coordinate v2 = mesh.vertex(tri.i2);
        RecFusion::Mesh::Coordinate v3 = mesh.vertex(tri.i3);

        // Calculate face normal
        RecFusion::Vec3 edge1(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
        RecFusion::Vec3 edge2(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
        RecFusion::Vec3 normal = RecFusion::Mat3::operator*(&edge1, &edge2);

        // Accumulate curvature for each vertex
        vertexCurvatures[tri.i1] += std::acos(Help_Functions::DotProduct(&normal, &edge1));
        vertexCurvatures[tri.i2] += std::acos(Help_Functions::DotProduct(&normal, &edge2));
        vertexCurvatures[tri.i3] += std::acos(Help_Functions::DotProduct(&normal, &RecFusion::Vec3(v1.x - v3.x, v1.y - v3.y, v1.z - v3.z)));
    }

    // Calculate average curvature
    return std::accumulate(vertexCurvatures.begin(), vertexCurvatures.end(), 0.0) / mesh.vertexCount();
}

static double calculateVolume(const RecFusion::Mesh& mesh) {
    double volume = 0.0;

    for (int i = 0; i < mesh.triangleCount(); ++i) {
        RecFusion::Mesh::Triangle tri = mesh.triangle(i);
        RecFusion::Mesh::Coordinate v1 = mesh.vertex(tri.i1);
        RecFusion::Mesh::Coordinate v2 = mesh.vertex(tri.i2);
        RecFusion::Mesh::Coordinate v3 = mesh.vertex(tri.i3);

        // Calculate signed volume of tetrahedron formed by triangle and origin
        volume += (v1.x * v2.y * v3.z + v2.x * v3.y * v1.z + v3.x * v1.y * v2.z -
            v1.x * v3.y * v2.z - v2.x * v1.y * v3.z - v3.x * v2.y * v1.z) / 6.0;
    }

    return std::abs(volume);
}

static double calculateSurfaceArea(const RecFusion::Mesh& mesh) {
    double area = 0.0;

    for (int i = 0; i < mesh.triangleCount(); ++i) {
        RecFusion::Mesh::Triangle tri = mesh.triangle(i);
        RecFusion::Mesh::Coordinate v1 = mesh.vertex(tri.i1);
        RecFusion::Mesh::Coordinate v2 = mesh.vertex(tri.i2);
        RecFusion::Mesh::Coordinate v3 = mesh.vertex(tri.i3);

        RecFusion::Vec3 edge1(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
        RecFusion::Vec3 edge2(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
        RecFusion::Vec3 crossProduct = Help_Functions::GetCrossProduct(&edge1, &edge2);

        area += 0.5 * std::sqrt(Help_Functions::DotProduct(&crossProduct, &crossProduct));
    }

    return area;
}
};
*/
/*
//bool MeshAnalyzer::needsDecimation(const RecFusion::Mesh& mesh, double ratioThreshold) {
//    double surfaceArea = calculateTriangleArea(mesh);
//    int triangleCount = mesh.triangleCount();
//    return (triangleCount / surfaceArea) > ratioThreshold;
//}
//
//bool MeshAnalyzer::needsHollowing(const RecFusion::Mesh& mesh, double ratioThreshold) {
//    double volume = calculateVolume(mesh);
//    double surfaceArea = calculateTriangleArea(mesh);
//    return (volume / surfaceArea) > ratioThreshold;
//}

bool MeshAnalyzer::needsBoundaryFaceRemoval(const RecFusion::Mesh& mesh, double ratioThreshold) {
    int boundaryFaces = countBoundaryFaces(mesh);
    int totalFaces = mesh.triangleCount();
    return (static_cast<double>(boundaryFaces) / totalFaces) > ratioThreshold;
}

std::vector<double> MeshAnalyzer::calculateComponentAreas(const RecFusion::Mesh& mesh) {
    return std::vector<double>{mesh.triangleCount() * 0.1};
}

double MeshAnalyzer::calculateAverageDihedralAngle(const RecFusion::Mesh& mesh) {
    double totalAngle = 0.0;
    int edgeCount = 0;
    for (int i = 0; i < mesh.triangleCount(); ++i) {
        RecFusion::Mesh::Triangle tri = mesh.triangle(i);
        RecFusion::Mesh::Coordinate v1 = mesh.vertex(tri.i1);
        RecFusion::Mesh::Coordinate v2 = mesh.vertex(tri.i2);
        RecFusion::Mesh::Coordinate v3 = mesh.vertex(tri.i3);

        // Calculate normal
        RecFusion::Vec3 normal = calculateNormal(v1, v2, v3);

        // For each edge, find adjacent triangle and calculate dihedral angle
        // This is a simplified version and would need to be expanded
        totalAngle += calculateDihedralAngle(normal, normal);
        edgeCount++;
    }
    return totalAngle / edgeCount;
}

RecFusion::Vec3 MeshAnalyzer::calculateNormal(const RecFusion::Mesh::Coordinate& v1,
    const RecFusion::Mesh::Coordinate& v2,
    const RecFusion::Mesh::Coordinate& v3) {
    RecFusion::Vec3 edge1(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
    RecFusion::Vec3 edge2(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
    // Cross product
    return RecFusion::Vec3(edge1[1] * edge2[2] - edge1[2] * edge2[1],
        edge1[2] * edge2[0] - edge1[0] * edge2[2],
        edge1[0] * edge2[1] - edge1[1] * edge2[0]);
}

double MeshAnalyzer::calculateDihedralAngle(const RecFusion::Vec3& n1, const RecFusion::Vec3& n2) {
    // Dot product
    double dot = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
    return std::acos(dot);
}
*/

//HoleFilling 
//needsHoleFilling come later after the initial HoleFilling succeed

bool MeshAnalyzer::needsHoleFilling(const RecFusion::Mesh& mesh, double ratioThreshold, int boundaryEdges, int totalEdges) {
    return (static_cast<double>(boundaryEdges) / totalEdges) > ratioThreshold; //Edge based Approach
}

//oder Typ std::pair<int, int>& countBoundaryEdges(const RecFusion::Mesh& mesh)?
std::pair<std::map<std::pair<int, int>, int>, std::map<std::pair<int, int>, int>> MeshAnalyzer::countBoundaryEdges(const RecFusion::Mesh& mesh) {
    std::map<std::pair<int, int>, int> Edges;
    std::map<std::pair<int, int>, int> BoundaryEdges;

    for (int i = 0; i < mesh.triangleCount(); ++i)
    {
        RecFusion::Mesh::Triangle tri = mesh.triangle(i);
        // Check if any indices are zero
        if (tri.i1 == 0 && tri.i2 == 0 && tri.i3 == 0) {
            std::cerr << "Warning: Triangle " << i << " has all indices set to zero." << std::endl;
            throw std::exception("Fehler, lese Console");
        }
        // if the edge (tri.i1, tri.i2) already exists in the Edges map. If it does, it sets its count to 1 and adds it to BoundaryEdges.
//  If it doesn't, it increments its count in Edges and removes it from BoundaryEdges if it was there.
        if (Edges.count(std::make_pair(tri.i1, tri.i2)) != 0) {
            Edges[std::make_pair(tri.i1, tri.i2)] = 1;
            BoundaryEdges[std::make_pair(tri.i1, tri.i2)] = 1;
        }
        else {
            Edges[std::make_pair(tri.i1, tri.i2)] = Edges[std::make_pair(tri.i1, tri.i2)] + 1;
            if (BoundaryEdges.count(std::make_pair(tri.i1, tri.i2)) != 0) {
                BoundaryEdges.erase(std::make_pair(tri.i1, tri.i2));
            }
        }

        if (Edges.count(std::make_pair(tri.i1, tri.i3)) != 0) {
            Edges[std::make_pair(tri.i1, tri.i3)] = 1;
            BoundaryEdges[std::make_pair(tri.i1, tri.i3)] = 1;
        }
        else {
            Edges[std::make_pair(tri.i1, tri.i3)] = Edges[std::make_pair(tri.i1, tri.i3)] + 1;
            if (BoundaryEdges.count(std::make_pair(tri.i1, tri.i3)) != 0) {
                BoundaryEdges.erase(std::make_pair(tri.i1, tri.i3));
            }
        }

        if (Edges.count(std::make_pair(tri.i3, tri.i2)) != 0) {
            Edges[std::make_pair(tri.i3, tri.i2)] = 1;
            BoundaryEdges[std::make_pair(tri.i3, tri.i2)] = 1;
        }
        else {
            Edges[std::make_pair(tri.i3, tri.i2)] = Edges[std::make_pair(tri.i3, tri.i2)] + 1;
            if (BoundaryEdges.count(std::make_pair(tri.i3, tri.i2)) != 0) {
                BoundaryEdges.erase(std::make_pair(tri.i3, tri.i2));
            }
        }

    }
   
    // Count boundary edges (edges with count == 1)
    int boundaryEdgeCount = 0;
    int edgesCount = 0;
    boundaryEdgeCount = BoundaryEdges.size();
    edgesCount = Edges.size();
    std::cout << "Number of boundary edges: " << boundaryEdgeCount << std::endl;
    std::cout << "Number of edges: " << edgesCount<< std::endl;
    return std::make_pair(BoundaryEdges, Edges);
}



// Function to calculate the area of holes in the mesh
/*
double MeshAnalyzer::calculateHoleArea(const RecFusion::Mesh& mesh, int boundaryEdgesSum)
{
    double totalHoleArea = 0.0;

    // Iterate through boundary edges to calculate the area of each hole
    for (int i = 0; i < boundaryEdgesSum; ++i)
    {
        RecFusion::Mesh::Triangle tri = mesh.triangle(i);
        RecFusion::Mesh::Coordinate v1 = mesh.vertex(tri.i1);
        RecFusion::Mesh::Coordinate v2 = mesh.vertex(tri.i2);
        RecFusion::Mesh::Coordinate v3 = mesh.vertex(tri.i3);

        if (isBoundaryEdge(tri.i1, tri.i2, mesh) ||
            isBoundaryEdge(tri.i2, tri.i3, mesh) ||
            isBoundaryEdge(tri.i3, tri.i1, mesh))
        {
            totalHoleArea += calculateTriangleArea(v1, v2, v3);
        }
    }

    return totalHoleArea;
}
*/


double MeshAnalyzer::calculateHoleArea(const RecFusion::Mesh& mesh, std::map<std::pair<int, int>, int> BoundaryEdges)
{
    double totalHoleArea = 0.0;
    std::vector<std::vector<int>> holes;
    std::map<std::pair<int, int>, int> visitedEdges;
    std::vector<int> holeVertices;

    int startVertex = 0;
    int currentVertex = 0;
    // Find connected boundary edges to form holes
    for (const auto& edge : BoundaryEdges) {
        if (visitedEdges.find(edge.first) != visitedEdges.end())
        {
            continue;
        }
        if (visitedEdges.find(edge.first) == visitedEdges.end()) {
            
            startVertex = edge.first.first;
            currentVertex = edge.first.second;
            holeVertices.push_back(startVertex);

            while (true) {
                holeVertices.push_back(currentVertex);
                visitedEdges[{holeVertices[holeVertices.size() - 2], currentVertex}] = 1;
                bool foundNextEdge = false;
                // Find next vertex in the hole
                for (const auto& nextEdge : BoundaryEdges) {
                    if (nextEdge.first.first == currentVertex && visitedEdges.find(nextEdge.first) == visitedEdges.end());
                    {
                        currentVertex = nextEdge.first.second;
                        foundNextEdge = true;
                        break;
                    }
                }

                if(!foundNextEdge || holeVertices.size() > mesh.vertexCount()) {
                    holeVertices.clear();
                    break;
                }
            }
                    /*if (nextEdge.first.first == currentVertex && nextEdge.first.second != holeVertices[holeVertices.size() - 2]) {
                        currentVertex = nextEdge.first.second;
                        break;
                    }*/
        
            if (holeVertices.size() > 2 && currentVertex == startVertex) {
                holes.push_back(std::move(holeVertices));
                std::cout << "Number of holes detected: " << holes.back().size() << std::endl;
            }
            holeVertices.clear();
        }
    }

    // Calculate area for each hole
    RecFusion::Mesh::Coordinate v0;
    double holeArea = 0.0;
    for (const auto& hole : holes) {
        v0 = mesh.vertex(hole[0]);
        for (size_t i = 1; i < hole.size() - 1; ++i) {
            RecFusion::Mesh::Coordinate v1 = mesh.vertex(hole[i]);
            RecFusion::Mesh::Coordinate v2 = mesh.vertex(hole[i + 1]);
            holeArea += calculateTriangleArea(v0, v1, v2);
        }
        totalHoleArea += holeArea;
    }

    std::cout << "Number of holes: " << holes.size() << std::endl;
    std::cout << "Total hole area: " << totalHoleArea << std::endl;
    return totalHoleArea;
}

//TriangleSurfaceArea korrekt
//double
//MeshAnalyzer::calculateTriangleArea
//(
//    const RecFusion::Mesh::Coordinate& v1,
//    const RecFusion::Mesh::Coordinate& v2,
//    const RecFusion::Mesh::Coordinate& v3)
//{
//    RecFusion::Mesh::Coordinate vec1 = v2 - v1;    
//    RecFusion::Mesh::Coordinate vec2 = v3 - v1;//substrahieren unmöglich
//    RecFusion::Mesh::Coordinate crossProduct = vec1.cross(vec2);´//Nicht existierte cross
//    return  0.5 * crossProduct.norm();//Nicht existierte norn
//    // Kreuzprodukt liefert das doppelte der Dreiecksfläche
//}
// 
double MeshAnalyzer::calculateTriangleArea(const RecFusion::Mesh::Coordinate& v1,
    const RecFusion::Mesh::Coordinate& v2,
    const RecFusion::Mesh::Coordinate& v3) {
        // Calculate triangle area using Heron's formula
        double a = distance(v1, v2);
        double b = distance(v2, v3);
        double c = distance(v3, v1);
        double s = (a + b + c) / 2.0;
       return std::sqrt(s * (s - a) * (s - b) * (s - c));
}

//Calculate Euclidean distance between two points korrekt
double MeshAnalyzer::distance(const RecFusion::Mesh::Coordinate& v1, const RecFusion::Mesh::Coordinate& v2) {
    double dx = v1.x - v2.x;
    double dy = v1.y - v2.y;
    double dz = v1.z - v2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

//double MeshAnalyzer::calculateVolume(const RecFusion::Mesh& mesh) {
//    return mesh.triangleCount() * 0.1;
//}

 

