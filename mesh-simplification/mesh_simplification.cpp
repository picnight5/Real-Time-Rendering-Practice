#include "mesh_simplification.h"
//-------------------------------------准备工作------------------------------


void updateFaces(int v1, int v2, std::vector<veci3>& faces,
    std::vector<std::vector<int>>& faces_of_vertices) {
    
    // 更新faces_of_vertices数组，移除与v1和v2相关的面索引记录，并添加与v3相关的新记录
    
    std::vector<int> facesContainingV1 = faces_of_vertices[v1];
    //std::vector<int> facesContainingV2 = faces_of_vertices[v2];
    faces_of_vertices[v1].clear();
    //faces_of_vertices[v2].clear();

    for (int faceIndex : facesContainingV1) {
       
            faces_of_vertices[v2].push_back(faceIndex);
        
    }
    /*for (int faceIndex : facesContainingV2) {
        
            faces_of_vertices[v3].push_back(faceIndex);
        
    }*/
    
    int deletedvertex;// 检查faces[thisface]的三个值中有几个被删顶点

    // 遍历faces_of_vertices[v3]，
    // 1.删去其中的非法面 2.更新faces中的顶点索引
    for (size_t thisfaceIndex = 0; thisfaceIndex < faces_of_vertices[v2].size(); /*递增有条件*/) {
       int thisface = faces_of_vertices[v2][thisfaceIndex];

        deletedvertex = 0;
        //更新faces中的顶点索引
        for (int j = 0; j < 3; ++j) {
            
                if ((faces[thisface][j] == v1) ||( faces[thisface][j] == v2)) {
                    faces[thisface][j] = v2;
                    deletedvertex++;
                }     
           
        }

        // 如果thisface包含v1和v2，则为非法面，删去faces_of_vertices[v3][thisfaceIndex] 元素
        if (deletedvertex ==2) {
            faces[thisface] = veci3(-1, -1, -1);//非法面赋值
            
            // 使用erase函数删除当前元素
            faces_of_vertices[v2].erase(faces_of_vertices[v2].begin() + thisfaceIndex);
            // 由于删除了一个元素，索引不需要递增，下一次循环会检查新的当前位置元素
        }
        else {
            // 如果没有重复值，正常递增索引，继续检查下一个元素
            ++thisfaceIndex;
        }
    }
}


vecf3 computeVnew(int v1, int v2, const std::vector<vecf3>& vertices, const std::vector<matf4>& qMatrices) {
    matf4 Q,Qnew2;
    matf4 inverseQnew2;
    vecf4 FourD_Vnew;
    vecf3 Vnew;
    Q = qMatrices[v1] + qMatrices[v2];
    Qnew2 << Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3),
        Q(0, 1), Q(1, 1), Q(1, 2), Q(1, 3),
        Q(0, 2), Q(1, 2), Q(2, 2), Q(2, 3),
        0, 0, 0, 1;

    if (Qnew2.determinant() != 0) {
        vecf4 m(0, 0, 0, 1);

        inverseQnew2 = Qnew2.inverse();
        FourD_Vnew = inverseQnew2 * m;
        Vnew[0] = FourD_Vnew.x();
        Vnew[1] = FourD_Vnew.y();
        Vnew[2] = FourD_Vnew.z();
    }
    else {
        Vnew = (vertices[v1] +vertices[v2]) * 0.5f;
    }

    return Vnew;
}

// 自定义比较函数，用于创建小顶堆
struct CompareEdge {
    bool operator()(const Edge& a, const Edge& b) const {
        return b < a; // 反转比较逻辑，使得小顶堆中较小的元素在顶部
    }
};
/*struct Edge {
    int v1;
    int v2;
    float cost;

    // 定义比较运算符，用于在优先队列中按照代价从小到大排序
    bool operator<(const Edge& other) const {
       return cost < other.cost;
    }
};*/
//----------------------------------------------------------------------------------

Model *simplify_mesh(
        const std::vector<vecf3>& _vertices,    // positions of vertices in the mesh
        const std::vector<veci3>& _faces,       // indices of vertices in each face
        float ratio                             // the ratio of the number of vertices after simplification to the original number of vertices
        ) {

    // avoid modifying the original mesh
    std::vector<vecf3> vertices = _vertices;
    std::vector<veci3> faces = _faces;

    // record whether the vertex is deleted
    std::deque<bool> vertices_deleted(vertices.size(), false);

    // record the face index of each vertex,
    std::vector<std::vector<int>> faces_of_vertices(vertices.size());
    for (int i = 0; i < faces.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            faces_of_vertices[faces[i][j]].push_back(i);
        }
    }
    for (int i = 0; i < vertices.size(); ++i) {
        if (faces_of_vertices[i].empty()) {
            vertices_deleted[i] = true;
        }
    }
   
    // TODO 3.1:
    // compute the Q matrices for all the initial vertices
    std::vector<matf4> qMatrices(vertices.size());
    matf4 Kp;
    matf4 Q;

    for (int j = 0; j < vertices.size(); ++j) {
        Q.setZero();

        for (int i = 0; i < faces_of_vertices[j].size(); ++i) {
            int thisface = faces_of_vertices[j][i];
            
            const vecf3& v0 = vertices[faces[thisface][0]];
            const vecf3& v1 = vertices[faces[thisface][1]];
            const vecf3& v2 = vertices[faces[thisface][2]];

            // 计算平面的法向量
            vecf3 edge1 = v1 - v0;
            vecf3 edge2 = v2 - v0;
            vecf3 normal = edge1.cross(edge2);

            // 归一化法向量
            if (normal.norm() > 0.0)
            {
                normal.normalize();

            }

            //face_normals[i] = normal;

            // 计算平面方程中的d参数
            float d = -normal.dot(v0);
            //face_ds[i] = -normal.dot(v0);
            //face_ds[i] = -(normal.x * v0.x + normal.y * v0.y + normal.z * v0.z);

            
            Kp << normal[0] * normal[0], normal[0] * normal[1], normal[0] * normal[2], normal[0] * d,
                normal[0] * normal[1], normal[1] * normal[1], normal[1] * normal[2], normal[1] * d,
                normal[0] * normal[2], normal[1] * normal[2], normal[2] * normal[2], normal[2] * d,
                normal[0] * d, normal[1] * d, normal[2] * d, d* d;

           
            Q = Q + Kp;
        }
        qMatrices[j] = Q;
    }

    //std::vector<vecf3> face_normals(faces.size());
    //std::vector<float> face_ds(faces.size());

    // TODO 3.2:        //注意！pair selection未考虑第二种情况，Vnew计算可完善！
    
    // select all valid pairs(edges) and compute the cost of each edge
    // 使用优先队列（堆）来存储边的代价信息，以便快速找到最小代价的边
    //std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> edgeHeap;
    //std::priority_queue<Edge> edgeHeap;
    std::priority_queue<Edge, std::vector<Edge>, CompareEdge> edgeHeap;
    // 用于辅助去重的集合，存储已经处理过的边的信息（以v1和v2组成的pair形式）
    std::set<std::pair<int, int>> processedEdges;

    for (int i = 0; i < faces.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v1 = faces[i][j];
            int v2 = faces[i][(j + 1) % 3];

            // 确保边的两个顶点顺序是固定的，以便后续处理
            if (v1 > v2) {
                std::swap(v1, v2);
            }

            // 检查这条边是否已经处理过
           if (processedEdges.find({ v1, v2 }) != processedEdges.end()) {
                continue;
            }

            // 计算边的代价
            vecf3 Vnew;
            Vnew=computeVnew(v1, v2, vertices, qMatrices);
           
            vecf4 D4Vnew;
            D4Vnew = vecf4(Vnew[0], Vnew[1], Vnew[2], 1);
           
            matf4 Qnew;
            Qnew = qMatrices[v1] + qMatrices[v2];
            Eigen::Matrix<float, 1, 1> result = D4Vnew.transpose() * Qnew * D4Vnew;

            float cost = result(0,0);

            Edge edge = { v1, v2, Vnew,cost };
            edgeHeap.push(edge);

            // 将处理过的边加入到已处理集合中
            processedEdges.insert({ v1, v2 });

        }
    }
    // 创建一个优先队列edgeHeap，其元素类型为EdgeCost结构体，按照代价从小到大排序（通过std::greater<EdgeCost>指定）。
    // 通过两层循环遍历面数组faces，对于每个面的三条边，获取边的两个顶点索引（确保顺序固定），
    // 然后调用computeEdgeCostEigen函数（需根据具体数学定义实现）根据这两个顶点的Q矩阵计算边的折叠代价，
    // 将边的信息（顶点索引和代价）封装成EdgeCost结构体并添加到优先队列edgeHeap中

   
   
    // TODO 3.3:
    // iteratively remove the pair of the least cost from the heap
    uint32_t face_cnt = faces.size();
    //uint32_t face_cnt = vertices.size();//=========================疑问：是否该这么写？
     
    uint32_t target_face_cnt = face_cnt * ratio;
    std::vector<Edge> temp;
    std::set<std::pair<int, int>> processedEdges2;

    while (face_cnt > target_face_cnt) {
        // remove the min edge from the heap
        Edge minEdge = edgeHeap.top();
        edgeHeap.pop();

        //vertices_deleted.resize(vertices_deleted.size() + 1, false);
        vertices_deleted[minEdge.first] = true;
        //vertices_deleted[minEdge.second] = true;
        
        // update the costs of all valid pairs
            //把Vnew加入vertices,Qnew加入qMatrices

        matf4 Qnew;
        Qnew = qMatrices[minEdge.first] + qMatrices[minEdge.second];

        //vertices.push_back(minEdge.vnew);
        vertices[minEdge.second] = minEdge.vnew;
        //qMatrices.push_back(Qnew);
        qMatrices[minEdge.second] = Qnew;

        //int v3 = vertices.size() - 1;
        //faces_of_vertices.resize(faces_of_vertices.size() + 1);
        //更新面数据
        updateFaces(minEdge.first, minEdge.second, faces, faces_of_vertices);

        //---------------------------更新堆---------------------
        
        // 用于辅助去重的集合，存储已经处理过的边的信息（以v1和v2组成的pair形式）
        
        temp.clear();
        processedEdges2.clear();
        while (!edgeHeap.empty()) {
            // 获取堆顶的边信息，即当前要处理的边
            Edge currentEdge = edgeHeap.top();

            // 检查当前边的顶点信息是否包含已删除的顶点
            // 如果不包含，说明这是不受影响的边，存进temp
            if (currentEdge.first != minEdge.first && currentEdge.first != minEdge.second && currentEdge.second != minEdge.first && currentEdge.second != minEdge.second) {
                temp.push_back(currentEdge);
            }
            else {
            int v1, v2;
                if (currentEdge.first == minEdge.first || currentEdge.first == minEdge.second) {

                 v1 = minEdge.second;
                 v2 = currentEdge.second;
                }
                else {//(currentEdge.v2 == minEdge.v1 || currentEdge.v2 == minEdge.v2)

                 v2 = minEdge.second;
                 v1 = currentEdge.first;
                 }

            // 确保边的两个顶点顺序是固定的，以便后续处理
            if (v1 > v2) {
                std::swap(v1, v2);
            }

            // 检查这条边是否已经处理过
            if (processedEdges2.find({ v1, v2 }) != processedEdges2.end()) {



                // 计算边的代价
                vecf3 Vnew;
                Vnew = computeVnew(v1, v2, vertices, qMatrices);

                vecf4 D4Vnew;
                D4Vnew = vecf4(Vnew[0], Vnew[1], Vnew[2], 1);

                matf4 Qnew;
                Qnew = qMatrices[v1] + qMatrices[v2];
                Eigen::Matrix<float, 1, 1> result = D4Vnew.transpose() * Qnew * D4Vnew;

                float cost = result(0, 0);

                Edge edge = { v1, v2, Vnew,cost };
                
                //新的边存进temp
                temp.push_back(edge);

                // 将处理过的边加入到已处理集合中
                processedEdges2.insert({ v1, v2 });
            }

            }

            // 将堆顶的边从堆中移除，以便处理下一个堆顶元素
            edgeHeap.pop();
        }

        for (auto it = temp.begin(); it != temp.end(); ++it) {
            edgeHeap.push(std::move(*it));
        }
        // 注意：在移动之后，temp 中的元素将处于有效但未定义的状态

        face_cnt -= 2;

    }


    // create the new mesh
    int new_vert_cnt = 0;
    int new_face_cnt = 0;
    for (auto i = 0; i < vertices.size(); ++i) {
        if (!vertices_deleted[i]) {
            vertices[new_vert_cnt] = vertices[i];
            for (auto face : faces_of_vertices[i]) {
                assert(face != -1);
                for (int j = 0; j < 3; ++j) {
                    if (faces[face][j] == i) {
                        faces[face][j] = new_vert_cnt;
                    }
                }
            }
            new_vert_cnt += 1;
        }
    }
    for (int i = 0; i < faces.size(); ++i) {
        if (faces[i][0] != faces[i][1] && faces[i][1] != faces[i][2] && faces[i][2] != faces[i][0]) {
            faces[new_face_cnt] = faces[i];
            new_face_cnt += 1;
        }
    }
    vertices.resize(new_vert_cnt);
    faces.resize(new_face_cnt);

    return Model::load(vertices, faces);
}
