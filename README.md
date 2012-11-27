Box2D-Typescript
================

Box2D definitions for TypeScript

== Naming changes

All names in lib was changed to match JavaScript/Typescript style. So functio
names start with lowercase letter. Constants are upcase. All prefices
"b2" and "E_" was removed (there are namespaces instead).

For example use

       var bodyDef = new Box2D.Dynamics.BodyDef();
       bodyDef.type = Box2D.Dynamics.Body.DYNAMIC_BODY;
       var body = world.createBody(bodyDef);

instead of

       var bodyDef = new Box2D.Dynamics.b2BodyDef();
       bodyDef.type = Box2D.Dynamics.b2Body.b2_dynamicbody;
       var body = world.CreateBody(bodyDef);
