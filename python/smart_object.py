from pyuipc_loader import pyuipc

SmartObjectA = pyuipc.SmartObjectA
SmartObjectB = pyuipc.SmartObjectB
create_smart_object = pyuipc.create_smart_object
receive_smart_object = pyuipc.receive_smart_object
view = pyuipc.view

soa0 = create_smart_object()
receive_smart_object(soa0)
print("soa0:", soa0)

soa1 = SmartObjectA()
receive_smart_object(soa1)
print("soa1:", soa1)

print(soa1.name())
print(soa1.view())
print(view(soa1))

sob = SmartObjectB()
print(sob.view())
print(view(sob))


geometry = pyuipc.geometry
SimplicialComplex = geometry.SimplicialComplex

sc = SimplicialComplex()
print(sc)


