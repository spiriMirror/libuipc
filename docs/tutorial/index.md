# Tutorial

Hey there! Welcome to the tutorial section of the Libuipc documentation. This section is designed to help you get started with Libuipc and understand the basics of the library. If you are new to Libuipc, this is the best place to start.

Most of the codes can be found in [libuipc-samples](https://github.com/spiriMirror/libuipc-samples).

## Hello Libuipc

This is a simple example to get you started with `libuipc`. In this example, we will create a simple simulation using Libuipc.

=== "C++"

    First, we include the `uipc.h` header file, which includes most the necessary headers for the `libuipc` library. Then, we use the namespace to make the code more readable:

    ```cpp
    #include <uipc/uipc.h>

    int main()
    {
        using namespace uipc;
        using namespace uipc::core;
        using namespace uipc::geometry;
        using namespace uipc::constitution;

        ...
    }
    ```

=== "Python"

    First, we import pyuipc, and alias some of the modules to make the code more readable:

    ```python
    import uipc
    from uipc import Vector3
    from uipc import Engine, World, Scene, SceneIO
    from uipc.geometry import *
    from uipc import builtin
    from uipc.unit import GPa, MPa

    import numpy as np
    ```

Then we create an instance of the `Engine` class, which is the main class of the `libuipc` simulation engine. We pass the `"cuda"` string to the constructor to specify the backend engine to use. 

`Engine` is the heart of the simulation, which drives the `World` evolution. 

`Scene` is a **snapshot** of the simulation at a certain time, and contains all information we needed to drive the simulation.

=== "C++"

    ```cpp
    #include <uipc/uipc.h>

    int main()
    {
        ...
        Engine engine{"cuda"};
        World  world{engine};
        auto config = Scene::default_config();
        config["dt"] = 0.01_s;
        config["gravity"] = Vector3{0, -9.8, 0};
        Scene scene{config};
    }
    ```

=== "Python"

    ```python
    engine = Engine('cuda')
    world = World(engine)
    config = Scene.default_config()
    config['dt'] = 0.01
    config['gravity'] = [[0.0], [-9.8], [0.0]]
    scene = Scene(config)
    ```

Here, we configure the `Scene` with a time step of $0.01s$ and a gravity of $9.8 m/s^2$ in the negative y-direction.

To evolve the simulation, we need setup the initial state of the simulation, i.e., the initial `Scene`.

In this example, we want to simulate a free-falling rigid tetrahedron bumping into another fixed rigid tetrahedron. So,

1. We need to apply a `AffineBodyConstitution` to the tetrahedra, to make them behave like rigid body.
2. We need to specify what kind of contact coefficient to use when the two tetrahedra collide.

> Constitution is a set of rules that govern the behavior of the objects in the simulation. In this case, we use `AffineBodyConstitution` to make the tetrahedra behave like a rigid body. Don't worry about the details of so-called **Constitution**, we will cover it in the later sections.


> Contact model is a set of rules that govern the interaction between two objects when they collide.


Before that, we should first add the `AffineBodyConstitution` to the `Scene`, and configure the contact model. `Libuipc` supports flexible configuration of the contact model, any pair of colliding objects can have their own contact model. For simplicity, in this example, we will use the default contact model for all the objects:

=== "C++"

    ```cpp
    #include <uipc/uipc.h>
    #include <uipc/constitutions/affine_body.h>

    int main()
    {
        ...
        Scene scene{config};
        {
            // create constitution and contact model
            AffineBodyConstitution abd;
            scene.contact_tabular().default_model(0.5, 1.0_GPa);
            auto& default_element = scene.contact_tabular().default_element();
            ...
        }
    }
    ```

=== "Python"

    ```python
    from uipc.constitution import AffineBodyConstitution
    ...
    scene = Scene(config)
    # create constitution and contact model
    abd = AffineBodyConstitution()
    scene.contact_tabular().default_model(0.5, 1.0 * GPa)
    default_element = scene.contact_tabular().default_element()
    ...
    ```

As you can see, we set the defualt contact model to have a friction ratio of $0.5$ and a contact resistance of $1.0GPa$, which is common in the real world. And after specifying the contact model, we create a `default_element` in order to apply the default contact model to the mesh in the later steps.

It's time to create the mesh! 

In this example, we will just manually create a regular tetrahedron (`base_mesh`), and apply a `AffineBodyConstitution` with a stiffness(hardness) of $100MPa$ to the `base_mesh`, which may be a hard-rubber-like material in the real world. Then we apply the default contact model to the `base_mesh`:

=== "C++"

    ```cpp
    int main()
    {
        ...
        Scene scene{config};
        {
            ...
            // create a regular tetrahedron
            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};
            vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

            // setup a base mesh to reduce the later work
            SimplicialComplex base_mesh = tetmesh(Vs, Ts);
            // apply the constitution model to the base mesh
            abd.apply_to(base_mesh, 100.0_MPa);
            // apply the default contact model to the base mesh
            default_element.apply_to(base_mesh);

            // label the surface, enable the contact
            label_surface(base_mesh);
            // label the triangle orientation to export the correct surface mesh
            label_triangle_orient(base_mesh);
        }
    }
    ...
    ```
=== "Python"

    ```python
    ...
    scene = Scene(config)
    ...
    # create a regular tetrahedron
    Vs = np.array([[0,1,0],
                   [0,0,1],
                   [-np.sqrt(3)/2, 0, -0.5],
                   [np.sqrt(3)/2, 0, -0.5]])
    Ts = np.array([[0,1,2,3]])

    # setup a base mesh to reduce the later work
    base_mesh = tetmesh(Vs, Ts)
    # apply the constitution and contact model to the base mesh
    abd.apply_to(base_mesh, 100 * MPa)
    # apply the default contact model to the base mesh
    default_element.apply_to(base_mesh)

    # label the surface, enable the contact
    label_surface(base_mesh)
    # label the triangle orientation to export the correct surface mesh
    label_triangle_orient(base_mesh)
    ```

> `SimplicialComplex` is an expressive geometry representation, which is well-defined in mathematics. `Libuipc` use the concept of `SimplicialComplex` to represent the geometry of the discrete mesh in the simulation. Don't worry about the details of `SimplicialComplex`, we will cover it in the [Geometry](./geometry.md). Now, just think of it as a container that holds the vertices/edges/triangles/tetrahedra of the mesh.

Keep in mind that, only we call `label_surface` on the mesh, will the engine consider the surface of the mesh for contact.

And `label_triangle_orient` is used to label the orientation of the triangles in the mesh, which is necessary for exporting the correct surface mesh. It doesn't affect the simulation itself.

Using the `base_mesh`, we can easily copy the setup to create two tetrahedra, `mesh1` and `mesh2`, and modify them as we like:

=== "C++"

    ```cpp
    int main()
    {
        ...
        Scene scene{config};
        {
            ...

            SimplicialComplex mesh1 = base_mesh;
            {
                // move the mesh1 up for 1.5 unit
                auto pos_view = view(mesh1.positions());
                std::ranges::transform(pos_view,
                                    pos_view.begin(),
                                    [](const Vector3& v) -> Vector3
                                    { return v + Vector3::UnitY() * 1.5; });
            }

            SimplicialComplex mesh2 = base_mesh;
            {
                // find the is_fixed attribute
                auto is_fixed = mesh2.instances().find<IndexT>(builtin::is_fixed);
                // set the first instance to be fixed
                auto is_fixed_view = view(*is_fixed);
                is_fixed_view[0]   = 1;
            }
        }
    }
    ```
=== "Python"

    ```python
    Scene scene = Scene(config)
    ...
    mesh1 = base_mesh.copy()
    pos_view = view(mesh1.positions())
    # move the mesh up for 1.5 unit
    pos_view += Vector3.UnitY() * 1.5

    mesh2 = base_mesh.copy()
    is_fixed = mesh2.instances().find(builtin.is_fixed)
    is_fixed_view = view(is_fixed)
    is_fixed_view[:] = 1
    ```

For `mesh1`, we move it up for $1m$ along the y-axis. And for `mesh2`, we set the first instance to be fixed.

In this section, we won't go into the details of the manipulation of the mesh (exactly, the geometry), we will cover it in the [Geometry](./geometry.md). Now, just think the code above is the `Libuipc`'s way to modify the geometry.

In `libuipc`, `Scene` contains some objects consisting of geometries. In this example, we create 2 objects, `object1` and `object2`, each with a mesh:

=== "C++"

    ```cpp
    int main()
    {
        Scene scene{config};
        {
            ...
            // create objects
            auto object1 = scene.objects().create("upper_tet");
            {
                object1->geometries().create(mesh1);
            }

            auto object2 = scene.objects().create("lower_tet");
            {
                object2->geometries().create(mesh2);
            }
        }
    }
    ```

=== "Python"

    ```python
    ...
    Scene scene = Scene(config)
    ...
    # create objects
    object1 = scene.objects().create("upper_tet")
    object1.geometries().create(mesh1)

    object2 = scene.objects().create("lower_tet")
    object2.geometries().create(mesh2)
    ...
    ```

From the **API**, we know that an object can have multiple geometries. It's sensible, because in the real world, an object can have multiple parts. E.g., a cloth can have multiple patches, a car can have multiple parts, etc. Object is a concept grouping the geometries together to make up a nameable entity, which is pretty intuitive for the user to build the world. But in this example, we just use one geometry for each object, for simplicity.


Now, we have setup the initial state of the `Scene`, we can pass it to the `World` for later simulation:

=== "C++"

    ```cpp
    int main()
    {
        ...
        World world{engine};
        ...
        Scene scene{config};
        ...

        world.init(scene);

        SceneIO sio{scene};
        sio.write_surface(fmt::format("scene_surface{}.obj", world.frame()));

        while(world.frame() < 100)
        {
            world.advance();
            world.retrieve();
            sio.write_surface(fmt::format("scene_surface{}.obj", world.frame()));
        }
    }
    ```

=== "Python"

    ```python
    ...
    world = World(engine)
    ...
    scene = Scene(config)
    ...
    
    world.init(scene)

    sio = SceneIO(scene)
    sio.write_surface(f"scene_surface{world.frame()}.obj")

    while world.frame() < 100:
        world.advance()
        world.retrieve()
        sio.write_surface(f"scene_surface{world.frame()}.obj")
    ```

To evolve the simulation, we call the `advance` method of the `World` to advance the simulation by one time step. Finally, we call the `retrieve` method to retrieve (download) the simulation data from the `World`.

The easiest way to consume the simulation data is to export the surface mesh of the `Scene` at each time step. Here we use the `SceneIO` class to export the surface of all the meshes to the `.obj` file.

<div align="center">
<video style="width:75%" muted="" controls="" alt="type:video">
   <source src="./media/falling_tet.mp4" type="video/mp4">
</video>
</div>

It's not the only way to consume the simulation data. Because the interest of the user may vary: some may want to visualize the simulation in some GUI software, some may want to analyze the simulation data in some post-processing software, some may want to do some machine learning on the simulation data, etc. It's up to you to decide what kind of data to get from the evolution of the `World`.

=== "C++"

    source: [TODO]

=== "Python"

    source: [hello_libuipc](https://github.com/spiriMirror/libuipc-samples/blob/main/python/1_hello_libuipc/main.py)

## Next Steps

Now you may be interested in the following topics:

1. [ [Geometry](./geometry.md) ] How can I understand the geometry in `libuipc`? 
2. [ [Concepts](./concepts.md) ] How can I understand the basic concepts in `libuipc`? 
3. [ [Animation](./animation.md) ] How can I script my own animation in `libuipc`?

It's recommended to read the above topics in order, as they are the basic concepts of `libuipc`.

