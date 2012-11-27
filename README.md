Box2D-Typescript
================

Box2D definitions for TypeScript

Source contains two files: TypeScript definitions file and JavaScript file. Definitions
are not allowed to be used with any different Box2DWeb source because all names in the library
was changed to match JavaScript/Typescript style.

So function names start with lowercase letter. Constants are upcase. All prefices "b2" and
"E_" was removed (there are namespaces instead).

For example use

       var bodyDef = new Box2D.Dynamics.BodyDef();
       bodyDef.type = Box2D.Dynamics.Body.DYNAMIC_BODY;
       var body = world.createBody(bodyDef);

instead of

       var bodyDef = new Box2D.Dynamics.b2BodyDef();
       bodyDef.type = Box2D.Dynamics.b2Body.b2_dynamicbody;
       var body = world.CreateBody(bodyDef);
