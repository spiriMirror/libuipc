#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/label_open_edge.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("label_open_edge", "[geometry]")
{
    // Test 1: Single triangle - all edges should be open
    {
        vector<Vector3> Vs = {
            Vector3{0, 0, 0},
            Vector3{1, 0, 0},
            Vector3{0, 1, 0}
        };
        vector<Vector3i> Fs = {
            Vector3i{0, 1, 2}
        };

        auto triangle = trimesh(Vs, Fs);
        auto is_open_attr = label_open_edge(triangle);
        auto is_open_view = is_open_attr->view();

        // All 3 edges should be open (shared by only 1 triangle)
        REQUIRE(is_open_view.size() == 3);
        REQUIRE(std::ranges::all_of(is_open_view, [](IndexT val) { return val == 1; }));
    }

    // Test 2: Two triangles sharing an edge - shared edge should be closed, others open
    {
        vector<Vector3> Vs = {
            Vector3{0, 0, 0},
            Vector3{1, 0, 0},
            Vector3{1, 1, 0},
            Vector3{0, 1, 0}
        };
        // Create two triangles: (0,1,2) and (0,2,3)
        // They share edge (0,2)
        vector<Vector3i> Fs = {
            Vector3i{0, 1, 2},
            Vector3i{0, 2, 3}
        };

        auto quad = trimesh(Vs, Fs);
        auto is_open_attr = label_open_edge(quad);
        auto is_open_view = is_open_attr->view();

        // Should have 5 edges: (0,1), (1,2), (2,0), (2,3), (3,0)
        REQUIRE(is_open_view.size() == 5);
        
        // Find edges by checking which ones are closed (shared by 2 triangles)
        // Only edge (0,2) should be closed
        auto edge_view = quad.edges().topo().view();
        IndexT closed_count = 0;
        IndexT open_count = 0;
        for(auto&& [i, is_open] : enumerate(is_open_view))
        {
            if(is_open == 0)
            {
                closed_count++;
                // Verify this is the shared edge (0,2) or (2,0)
                Vector2i e = edge_view[i];
                REQUIRE((e == Vector2i{0, 2} || e == Vector2i{2, 0}));
            }
            else
            {
                open_count++;
            }
        }
        REQUIRE(closed_count == 1);  // Only one closed edge
        REQUIRE(open_count == 4);    // Four open edges
    }

    // Test 3: Closed mesh (tetrahedron surface) - all edges should be closed
    {
        // Create a tetrahedron and extract its surface
        vector<Vector3> Vs = {
            Vector3{0, 0, 0},
            Vector3{1, 0, 0},
            Vector3{0.5, 1, 0},
            Vector3{0.5, 0.5, 1}
        };
        vector<Vector4i> Ts = {
            Vector4i{0, 1, 2, 3}
        };

        auto tet = tetmesh(Vs, Ts);
        label_surface(tet);
        auto surface = extract_surface(tet);

        auto is_open_attr = label_open_edge(surface);
        auto is_open_view = is_open_attr->view();

        // All edges should be closed (shared by 2 triangles each)
        REQUIRE(std::ranges::all_of(is_open_view, [](IndexT val) { return val == 0; }));
    }
}

