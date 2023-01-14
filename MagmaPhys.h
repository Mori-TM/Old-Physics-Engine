struct Transform
{
	glm::vec3 Position;
	glm::vec3 Scale;
	glm::vec4 Rotation;
};

struct Rigidbody
{
	float Mass;
	glm::vec3 Velocity;
	glm::vec3 Force;

	Transform Transform;
};

struct Sphere
{
	glm::vec3 Pos;
	float radius;
};

struct Plane
{
	glm::vec3 Normal;
	float yPos;
};

glm::vec3 RotateCOx(float angle)
{
	float ConvertedAngle = (angle+90)*-1 * PI / 180;

	return glm::normalize(glm::vec3(0.0, sin(ConvertedAngle), cos(ConvertedAngle)));
}

glm::vec3 RotateCOz(float angle)
{
	float ConvertedAngle = (angle-90) * PI / 180;

	return glm::normalize(glm::vec3(cos(ConvertedAngle), sin(ConvertedAngle), 0.0));
}

double distanceBetweenTwoPoints(float x, float y, float z, float xx, float yy, float zz) 
{
	return sqrt(pow(x - xx, 2) + pow(y - yy, 2) + pow(z - zz, 2));
}

void Collide(std::vector<Rigidbody*> plane, std::vector<Rigidbody*> DstObject)
{
	glm::vec3 Normal;

	for (int p = 0; p < plane.size(); p++)
	{
		if (plane[p]->Transform.Rotation.y != 0.0)
		{
			Normal = RotateCOx(plane[p]->Transform.Rotation.x);
		}
		else if (plane[p]->Transform.Rotation.w != 0.0)
		{
			Normal = RotateCOz(plane[p]->Transform.Rotation.x);
		}
		else
		{
			Normal = RotateCOx(0.0);
		}

		for (int i = 0; i < DstObject.size(); i++)
		{
			float res = std::abs(DstObject[i]->Transform.Position.x * Normal.x + DstObject[i]->Transform.Position.y * Normal.y + DstObject[i]->Transform.Position.z * Normal.z + plane[p]->Transform.Position.y);
		
			if (res <= DstObject[i]->Transform.Scale.x)
			{
			//	printf("SVP collision\n");
				
				float wl = sqrt(Normal.x * Normal.x + Normal.y * Normal.y + Normal.z * Normal.z);

				float nx = Normal.x / wl;
				float ny = Normal.y / wl;
				float nz = Normal.z / wl;

				float scope = (DstObject[i]->Velocity.x * nx + DstObject[i]->Velocity.y * ny + DstObject[i]->Velocity.z * nz) * 2;

				nx = nx * scope;
				ny = ny * scope;
				nz = nz * scope;

				DstObject[i]->Velocity.x -= nx;
				DstObject[i]->Velocity.y -= ny;
				DstObject[i]->Velocity.z -= nz;
				
			}
		}
	}	
}

struct vec3
{
	float x, y, z;
	vec3 operator- (const vec3& rhs) const { return{ x - rhs.x, y - rhs.y, z - rhs.z }; }
	float operator* (const vec3& rhs) const { return{ x * rhs.x + y * rhs.y + z * rhs.z }; } // DOT PRODUCT
	vec3 operator^ (const vec3& rhs) const { return{ y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x }; } // CROSS PRODUCT
	vec3 operator* (const float& rhs)const { return vec3{ x * rhs, y * rhs, z * rhs }; }
};

// set the relevant elements of our oriented bounding box
struct OBB
{
	vec3 Pos, AxisX, AxisY, AxisZ, Half_size;
};

// check if there's a separating plane in between the selected axes
bool getSeparatingPlane(const vec3& RPos, const vec3& Plane, const OBB& box1, const OBB& box2)
{
	return (fabs(RPos * Plane) >
		(fabs((box1.AxisX * box1.Half_size.x) * Plane) +
		 fabs((box1.AxisY * box1.Half_size.y) * Plane) +
		 fabs((box1.AxisZ * box1.Half_size.z) * Plane) +
		 fabs((box2.AxisX * box2.Half_size.x) * Plane) +
		 fabs((box2.AxisY * box2.Half_size.y) * Plane) +
		 fabs((box2.AxisZ * box2.Half_size.z) * Plane)));
}

bool getCollision(const OBB& box1, const OBB& box2)
{
	static vec3 RPos;
	RPos = box2.Pos - box1.Pos;

	return !(getSeparatingPlane(RPos, box1.AxisX, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisY, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisZ, box1, box2) ||
		getSeparatingPlane(RPos, box2.AxisX, box1, box2) ||
		getSeparatingPlane(RPos, box2.AxisY, box1, box2) ||
		getSeparatingPlane(RPos, box2.AxisZ, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisX ^ box2.AxisX, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisX ^ box2.AxisY, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisX ^ box2.AxisZ, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisY ^ box2.AxisX, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisY ^ box2.AxisY, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisY ^ box2.AxisZ, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisZ ^ box2.AxisX, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisZ ^ box2.AxisY, box1, box2) ||
		getSeparatingPlane(RPos, box1.AxisZ ^ box2.AxisZ, box1, box2));
}

void CollideBox(std::vector<Rigidbody*> Box)
{
	

	if (Box.size() > 1)
	{
		for (int j = 0; j < Box.size(); j++)
		{
			for (int i = j + 1; i < Box.size(); i++)
			{
				OBB A, B;

				// set the first obb's properties
				A.Pos = { Box[j]->Transform.Position.x, Box[j]->Transform.Position.y, Box[j]->Transform.Position.z }; // set its center position

				// set the half size
				A.Half_size.x = Box[j]->Transform.Scale.x;
				A.Half_size.y = Box[j]->Transform.Scale.y;
				A.Half_size.z = Box[j]->Transform.Scale.z;

				// set the axes orientation
				A.AxisX = { 1.f, 0.f, 0.f };
				A.AxisY = { 0.f, 1.f, 0.f };
				A.AxisZ = { 0.f, 0.f, 1.f };

				// set the second obb's properties
				B.Pos = { Box[i]->Transform.Position.x, Box[i]->Transform.Position.y, Box[i]->Transform.Position.z }; // set its center position

				// set the half size
				B.Half_size.x = Box[i]->Transform.Scale.x;
				B.Half_size.y = Box[i]->Transform.Scale.y;
				B.Half_size.z = Box[i]->Transform.Scale.z;

				// set the axes orientation
				B.AxisX = { 1.f, 0.f, 0.f };
				B.AxisY = { 0.f, 1.f, 0.f };
				B.AxisZ = { 0.f, 0.f, 1.f };

				if (getCollision(A, B)) std::cout << "Collision!!!" << std::endl;
			//	else std::cout << "No collision." << std::endl;
			}
		}

	}
	else
	{
		printf("No 2 Boxes To Collide!\n");
	}
}

void Collide(std::vector<Rigidbody*> Sphere)
{
	if (Sphere.size() > 1)
	{	
		for (int j = 0; j < Sphere.size(); j++)
		{
			for (int i = j + 1; i < Sphere.size(); i++)
			{

				float distance = sqrt((Sphere[j]->Transform.Position.x - Sphere[i]->Transform.Position.x) * (Sphere[j]->Transform.Position.x - Sphere[i]->Transform.Position.x) +
									  (Sphere[j]->Transform.Position.y - Sphere[i]->Transform.Position.y) * (Sphere[j]->Transform.Position.y - Sphere[i]->Transform.Position.y) +
									  (Sphere[j]->Transform.Position.z - Sphere[i]->Transform.Position.z) * (Sphere[j]->Transform.Position.z - Sphere[i]->Transform.Position.z));


				if (distance < (Sphere[j]->Transform.Scale.x + Sphere[i]->Transform.Scale.x))
				{
					glm::vec3 x = Sphere[j]->Transform.Position - Sphere[i]->Transform.Position;
					x = glm::normalize(x);

					float x1, m1, x2, m2;

					glm::vec3 v1, v1x, v1y, v2, v2x, v2y;
					v1 = Sphere[j]->Velocity;
					x1 = glm::dot(x, v1);
					v1x = x * x1;
					v1y = v1 - v1x;
					m1 = Sphere[j]->Mass;

					x = glm::vec3(x.x * -1, x.y * -1, x.z * -1);
					v2 = Sphere[i]->Velocity;
					x2 = glm::dot(x, v2);
					v2x = x * x2;
					v2y = v2 - v2x;
					m2 = Sphere[i]->Mass;

					Sphere[j]->Velocity = glm::vec3(v1x * (m1 - m2) / (m1 + m2) + v2x * (2 * m2) / (m1 + m2) + v1y);
					Sphere[i]->Velocity = glm::vec3(v1x * (2 * m1) / (m1 + m2) + v2x * (m2 - m1) / (m1 + m2) + v2y);
				}
			}
		}
		
	}
	else
	{
		printf("No 2 Sphere To Collide!\n");
	}
}

class PhysicsWorld
{
private:
	std::vector<Rigidbody*> ObjectList;
	glm::vec3 Gravity = glm::vec3(0.0, -9.8, 0.0);

public:
	
	void Step(float dt)
	{
		for (Rigidbody* obj : ObjectList)
		{
			obj->Force += obj->Mass * Gravity;

			obj->Velocity += obj->Force / obj->Mass * dt;
			obj->Transform.Position += obj->Velocity * dt;

			obj->Force = glm::vec3(0.0);
		}
	}
	
	void AddObject(Rigidbody* object)
	{
		ObjectList.push_back(object);
	}
	void RemoveObject(Rigidbody* object)
	{
		if (!object) return;
		auto itr = std::find(ObjectList.begin(), ObjectList.end(), object);
		if (itr == ObjectList.end()) return;
		ObjectList.erase(itr);
	}
};