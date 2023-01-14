# Old-Physics-Engine
This is the code for the physics engine from one of my old videos: https://youtu.be/835udxAKphg
Note: This code is old an not that good, I made some questional decisions here.
```C
Rigidbody SphereObjects[2048];

Rigidbody Plane0;
Rigidbody Plane1;

std::vector<Rigidbody*> Spheres;
std::vector<Rigidbody*> Planes;

PhysicsWorld SphereWorld;// Dynamic objects

void Init()
{
  for (int z = 0; z < 2048; z += 1)
  {
    SphereWorld.AddObject(&SphereObjects[z]);
    SphereObjects[z].Mass = 0.1;
    SphereObjects[z].Velocity = glm::vec3(0.0, 0.1, 0.0);
    SphereObjects[z].Force = glm::vec3(0.0, 0.0, 0.0);
    SphereObjects[z].Transform.Position = glm::vec3(-34.0, ((z * 8.0) + 20) / 2, 14.0 * sin(z * 6.0));
    SphereObjects[z].Transform.Scale = glm::vec3(1.5);
    Spheres.push_back(&SphereObjects[z]);
  }


  Plane0.Mass = 0.01;
  Plane0.Velocity = glm::vec3(0.1);
  Plane0.Force = glm::vec3(0.0);
  Plane0.Transform.Position = glm::vec3(0.0f, -0.01f, 0.0f);
  Plane0.Transform.Rotation = glm::vec4(0.0, 0.0, 0.0, 1.0);

  Plane1.Mass = 0.01;
  Plane1.Velocity = glm::vec3(0.1);
  Plane1.Force = glm::vec3(0.0);
  Plane1.Transform.Position = glm::vec3(0.0f, -10.0f, 0.0f);
  Plane1.Transform.Rotation = glm::vec4(-25.0, 0.0, 0.0, 1.0);

  Planes.push_back(&PlaneObject);
  Planes.push_back(&PlaneObject1);
}

void Update()
{
  SphereWorld.Step(DeltaTime() * FallSpeed);
  Collide(Planes, Spheres);
  Collide(Spheres);
  
  for (int z = 0; z < HeadCount; z += 1)
  {
    Translate(SphereObjects[z].Transform.Position.x, SphereObjects[z].Transform.Position.y, SphereObjects[z].Transform.Position.z);
    Scale(1.5, 1.5, 1.5);
    DrawSphere();
  }
}

```
