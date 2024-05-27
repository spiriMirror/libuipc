# Object

Object in `libuipc` is a representation of the concerate entity in real world. It is something that touchable, visible, and can be interacted with. For example, a toy bus. An object is a collection of [geometries](geometry.md) that define its behavior and appearance.

$$
\text{Object}_i = \{ \text{Geometry}_j \mid j \in [0, N^{i})\} 
$$

In this senario, an object is a collection of geometries. A geometry is an element of an object. For better comprehension, we call such kind of geometry as a component.

!!!note
    If a geometry belongs to an object, then we call it a component of the object, else we just call it a geometry.

!!!tip
    We don't have a `Component` class in `libuipc`, it is just a concept to help you understand the relationship between an object and its geometries.

The main constraint of a component in `libuipc` is that, it can only have one [constitution](#constitution) and one [contact model](#contact-model).

- Constitution is a set of properties that define the deformation behavior of the object. For example, a famous constitution is the [Neo-Hookean](https://en.wikipedia.org/wiki/Neo-Hookean_solid) model, which is used to simulate the deformation of rubber-like materials.
- Contact model is a set of properties that define the contact behavior of the object. Typically, the properties include the friction coefficient, the restitution coefficient, etc.

Though a component share the same constitution and contact model, the **coefficients** of the constitution and the contact model can be different in the geometry, thanks to `libuipc`'s geometry attribute system.

!!!info
    The coefficients of the constitution and the contact model can be stored in the attributes of the geometry. The backend can retrieve the coefficients from the attributes and simulate them properly.

To create an object, you can do the following:

```cpp
uipc::world::Scene scene;
auto& neo_hookean = scene.create<Constitution>("neo_hookean");

auto& wood_contact = scene.create<ContactModel>("wood"); // contact model id = 0;
auto& rubber_contact = scene.create<ContactModel>("rubber"); // contact model id = 1;
auto& tabular = scene.contact_tabular();

// self contact
tabular.insert(contact_model1, resitution, friction);
// contact with other object
tabular.insert(contact_model1, contact_model2, resitution, friction);

SimplicialComplexIO io;
auto cube = io.read("cube.msh");

wood_contact.apply_to(cube);
neo_hookean.apply_to(cube);

auto& object = scene.create<Object>();
{
    object.push_back(cube);
}
```

## Constitution

## Contact Model
