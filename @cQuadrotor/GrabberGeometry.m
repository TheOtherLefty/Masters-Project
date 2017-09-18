function GPos = GrabberGeometry(obj,X)

GPosBody = [0 0 obj.ArmLength]';
GPos = obj.TransformtoEarthAxes(GPosBody,X(1:3),X(4:6));