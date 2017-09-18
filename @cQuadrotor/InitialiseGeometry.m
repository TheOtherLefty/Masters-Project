function Geo = InitialiseGeometry(obj)

load('QuadrotorGeoModel.mat')
Geo = GeoModel;
Geo.BodyVertices = GeoModel.Vertices;