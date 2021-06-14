#include "boids.h"
#include <iostream>

float Boid::Range_FAttract = 25.0f; 
float Boid::Range_FRepel = 12.0f; 
float Boid::Range_FAlign = 15.0f; 
float Boid::FAttract_Vmax = 8.0f; 
float Boid::FAttract_Factor = 4.0f; 
float Boid::FRepel_Factor = 8.0f; 
float Boid::FAlign_Factor = 4.0f;
float Boid::minSpeed = 15.0f;
float Boid::maxSpeed = 50.0f;


///Constructor
Boid::Boid()
{
	pNode = nullptr;
	pRigidBody = nullptr;
	pCollisionShape = nullptr;
	pObject = nullptr;
	id = 0;
}
//Destructor
Boid::~Boid()
{
}

void Boid::SetID(int ID)
{
	id = ID;
}
int Boid::GetID()
{
	return id;
}
bool Boid::GetEnabled()
{
	return pNode->IsEnabled();
}

void Boid::Initialise(ResourceCache* pRCache, Scene* pScene, std::string n)
{
	name = n;
	pNode = pScene->CreateChild("Boid");
	pNode->SetPosition(Vector3(Random(800.0f) - 400.0f, Random(80.0f) + 10.0f, Random(800.0f) - 400.0f));
	pNode->SetRotation(Quaternion(0.0f, Random(360.0f), 0.0f));
	pNode->SetScale(0.5f);

	pObject = pNode->CreateComponent<StaticModel>();
	std::string modelPath = "Models/"+name + ".mdl";
	std::string materialPath = "Materials/" + name + ".xml";
	if(name=="ichthyosaur")
	{
		pObject->SetModel(pRCache->GetResource<Model>("Models/ichthyosaur.mdl"));
		pObject->SetMaterial(pRCache->GetResource<Material>("Materials/ichthyosaur.xml"));
	}

	else if (name == "plesiosaur")
	{
		pObject->SetModel(pRCache->GetResource<Model>("Models/plesiosaur.mdl"));
		pObject->SetMaterial(pRCache->GetResource<Material>("Materials/plesiosaur.xml"));
	}
	else
	{
		pObject->SetModel(pRCache->GetResource<Model>("Models/Cone.mdl"));
	    pObject->SetMaterial(pRCache->GetResource<Material>("Materials/Stone.xml"));
	}
	
	
	pObject->SetCastShadows(true);

	pRigidBody = pNode->CreateComponent<RigidBody>();
	pRigidBody->SetCollisionLayer(2);
	pRigidBody->SetMass(0.5f);
	pRigidBody->SetUseGravity(false);
	pRigidBody->SetPosition(Vector3(Random(800.0f) - 400.0f, Random(80.0f) + 10.0f, Random(800.0f) - 400.0f));
	pRigidBody->SetLinearVelocity(Vector3(Random(40.0f)-20.0f, 0.0f, Random(40.0f)-20.0f));

	pCollisionShape = pNode->CreateComponent<CollisionShape>();
	pCollisionShape->SetBox(Vector3::ONE);
	//pCollisionShape->SetTriangleMesh(pObject->GetModel(), 0);

}

/*void Boid::ComputeForce(Boid* boi, int size, Projectile* proj)
{
	force = Vector3(0, 0, 0);
	Attract(boi, size);
	Repel(boi, size);
	Repel(proj);
	Align(boi, size);
}
*/
void Boid::ResetForce()
{
	force = Vector3(0, 0, 0);
}
void Boid::ComputeForce(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi, Projectile* proj)
{
	//force = Vector3(0, 0, 0);
	Attract(boi);
	Repel(boi);
	Repel(proj);
	Align(boi);
}
void Boid::Attract(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi)
{
	Vector3 CoM; //centre of mass
	int n = 0;	//number of neighbours
	//int s = boi.size();
	int size;
	int X, Y ,Z;

	try {
		for (int x = -1; x <= 1; x++)
		{
			X = (int)(partition.x_) + (int)x;
			for (int y = -1; y <= 1; y++)
			{
				Y = (int)(partition.y_) + (int)y;
				for (int z = -1; z <= 1; z++)
				{
					Z = (int)(partition.z_) + z;
					//std::cout << "Indexes: " << X << " " << Y << " " << Z << "\n";
					
					if ((partition.x_ + x >= 0 && partition.x_ + x <= partitionsX - 1) && (partition.y_ + y >= 0 && partition.y_ + y <= partitionsY - 1) && (partition.z_ + z >= 0 && partition.z_ + z <= partitionsZ - 1))
					{
						size = boi[X][Y][Z].size();
						for (int i = 0; i < size; i++)
						{
							if (id == boi[X][Y][Z][i].GetID()) continue; //query if looking at current boid
							if (!boi[X][Y][Z][i].GetEnabled()) continue; //ignore disabled boids
							Vector3 diff = pRigidBody->GetPosition() - boi[X][Y][Z][i].pRigidBody->GetPosition(); //get the difference
							float dist = diff.Length();	//get distance from difference

							if (dist < Range_FAttract)
							{
								CoM += boi[X][Y][Z][i].pRigidBody->GetPosition();
								n++;
							}
						}
					}
				}
			}
		}
	}
	catch(std::exception & e)
	{
		std::cout << "Indexes: " << X << " " << Y << " " << Z << "\n";
	}
	/*
	for (int i = 0; i < size; i++)
	{
		if (this == &boi[i]) continue; //query if looking at current boid
		Vector3 diff = pRigidBody->GetPosition() - boi[i].pRigidBody->GetPosition(); //get the difference
		float dist = diff.Length();	//get distance from difference

		if (dist < Range_FAttract)
		{
			CoM += boi[i].pRigidBody->GetPosition();
			n++;
		}
	}*/
	if (n > 0)
	{
		CoM /= n;
		Vector3 direction = (CoM - pRigidBody->GetPosition()).Normalized();
		Vector3 vDesired = direction * FAttract_Vmax; //desired vector
		force += (vDesired - pRigidBody->GetLinearVelocity()) * FAttract_Factor;
	}
}

void Boid::Repel(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi)
{
	Vector3 CoM; //centre of mass
	int n = 0;	//number of neighbours

	int size;
	int X, Y, Z;

	try {
		for (int x = -1; x <= 1; x++)
		{
			X = (int)(partition.x_) + (int)x;
			for (int y = -1; y <= 1; y++)
			{
				Y = (int)(partition.y_) + (int)y;
				for (int z = -1; z <= 1; z++)
				{
					Z = (int)(partition.z_) + (int)z;
					
					if ((partition.x_ + x >= 0 && partition.x_ + x <= partitionsX - 1) && (partition.y_ + y >= 0 && partition.y_ + y <= partitionsY - 1) && (partition.z_ + z >= 0 && partition.z_ + z <= partitionsZ - 1))
					{
						size = boi[X][Y][Z].size();
						for (int i = 0; i < size; i++)
						{
							if (id == boi[X][Y][Z][i].GetID()) continue; //query if looking at current boid
							if (!boi[X][Y][Z][i].GetEnabled()) continue; //ignore disabled boids
							Vector3 diff = pRigidBody->GetPosition() - boi[X][Y][Z][i].pRigidBody->GetPosition(); //get the difference
							float dist = diff.Length();	//get distance from difference

							if (dist < Range_FRepel)
							{
								CoM += diff.Normalized();
								n++;
							}
						}
					}
				}
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << "Indexes: " << X << " " << Y << " " << Z << "\n";
	}
	/*
	for (int i = 0; i < size; i++)
	{
		if (this == &boi[i]) continue; //query if looking at current boid
		Vector3 diff = pRigidBody->GetPosition() - boi[i].pRigidBody->GetPosition(); //get the difference
		float dist = diff.Length();	//get distance from difference

		if (dist < Range_FRepel)
		{
			CoM += diff.Normalized();
			n++;
		}
	}
	*/
	if (n > 0)
	{
		//Vector3 direction = CoM;
		force += CoM * FRepel_Factor;
	}
}

void Boid::Repel(Projectile* proj)
{
	Vector3 CoM; //centre of mass
	Vector3 diff = pRigidBody->GetPosition() - proj->getRB()->GetPosition(); //get the difference
	float dist = diff.Length();	//get distance from difference

	if (dist < 4.0f)
	{
		//CoM += diff.Normalized();
		pNode->SetEnabled(false);
	}

	//force += CoM * FRepel_Factor * 2.0f;
}

void Boid::Align(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi)
{
	Vector3 CoM; //centre of mass
	int n = 0;	//number of neighbours
	int size;
	int X, Y, Z;

	try {
		for (int x = -1; x <= 1; x++)
		{
			int X = (int)(partition.x_) + (int)x;
			for (int y = -1; y <= 1; y++)
			{
				int Y = (int)(partition.y_) + (int)y;
				for (int z = -1; z <= 1; z++)
				{
					int Z = (int)(partition.z_) + (int)z;
					
					if ((partition.x_ + x >= 0 && partition.x_ + x <= partitionsX - 1) && (partition.y_ + y >= 0 && partition.y_ + y <= partitionsY - 1) && (partition.z_ + z >= 0 && partition.z_ + z <= partitionsZ - 1))
					{
						size = boi[X][Y][Z].size();
						for (int i = 0; i < size; i++)
						{
							if (id == boi[X][Y][Z][i].GetID()) continue; //query if looking at current boid
							if (!boi[X][Y][Z][i].GetEnabled()) continue; //ignore disabled boids
							Vector3 diff = pRigidBody->GetPosition() - boi[X][Y][Z][i].pRigidBody->GetPosition(); //get the difference
							float dist = diff.Length();	//get distance from difference

							if (dist < Range_FAlign)
							{
								CoM += diff.Normalized();
								n++;
							}
						}
					}
				}
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << "Indexes: " << X << " " << Y << " " << Z << "\n";
	}
	/*
	for (int i = 0; i < size; i++)
	{
		if (this == &boi[i]) continue; //query if looking at current boid
		Vector3 diff = pRigidBody->GetPosition() - boi[i].pRigidBody->GetPosition(); //get the difference
		float dist = diff.Length();	//get distance from difference

		if (dist < Range_FAlign)
		{
			CoM += diff.Normalized();
			n++;
		}
	}
	*/
	if (n > 0)
	{
		CoM /= n;
		Vector3 direction = CoM;
		force += (direction - pRigidBody->GetLinearVelocity())* FRepel_Factor;
	}
}

void Boid::Update(float time, Projectile* proj)
{
	

	pRigidBody->ApplyForce(force);

	Vector3 velocity = pRigidBody->GetLinearVelocity();
	float d = velocity.Length();
	if (d < minSpeed)
	{
		d = minSpeed;
		pRigidBody->SetLinearVelocity(velocity.Normalized() * d);
	}
	else if (d > maxSpeed)
	{
		d = maxSpeed;
		pRigidBody->SetLinearVelocity(velocity.Normalized() * d);
	}

	Vector3 velnorm = velocity.Normalized();
	Vector3 crossprod = -velnorm.CrossProduct(Vector3(0.0f, 1.0f, 0.0f));
	float dp = crossprod.DotProduct(velnorm);
	pRigidBody->SetRotation(Quaternion(Acos(dp), crossprod));

	Vector3 pos = pRigidBody->GetPosition();

	if (pos.y_ < worldBounds[1][0])
	{
		pos.y_ = worldBounds[1][0];
		pRigidBody->SetPosition(pos);
		pRigidBody->SetRotation(-Quaternion(Acos(dp), crossprod));
	}
	else if (pos.y_ > worldBounds[1][1])
	{
		pos.y_ = worldBounds[1][1];
		pRigidBody->SetPosition(pos);
		pRigidBody->SetRotation(-Quaternion(Acos(dp), crossprod));
	}
	if (pos.z_ < worldBounds[2][0])
	{
		pos.z_ = worldBounds[2][0];
		pRigidBody->SetPosition(pos);
		pRigidBody->SetRotation(-Quaternion(Acos(dp), crossprod));
	}
	else if (pos.z_ > worldBounds[2][1])
	{
		pos.z_ = worldBounds[2][1];
		pRigidBody->SetPosition(pos);
		pRigidBody->SetRotation(-Quaternion(Acos(dp), crossprod));
	}
	if (pos.x_ < worldBounds[0][0])
	{
		pos.x_ = worldBounds[0][0];
		pRigidBody->SetPosition(pos);
		pRigidBody->SetRotation(-Quaternion(Acos(dp), crossprod));
	}
	else if (pos.x_ > worldBounds[0][1])
	{
		pos.x_ = worldBounds[0][1];
		pRigidBody->SetPosition(pos);
		pRigidBody->SetRotation(-Quaternion(Acos(dp), crossprod));
	}


}

void Boid::UpdateCurrentPartition()
{
	Vector3 pos = pRigidBody->GetPosition();
	int X = (int)(((worldBounds[0][0] * -1) + pos.x_) / partitionSize);
	int Y = (int)(((worldBounds[1][0]) + pos.y_) / partitionSize); //y is 0 or > 0 so no need to multiply by -1
	int Z = (int)(((worldBounds[2][0] * -1) + pos.z_) / partitionSize);

	partition = Vector3(X, Y, Z);
	//std::cout << "Updated Partition: " << X << " " << Y << " " << Z << "\n";
}

Vector3 Boid::GetPartition()
{
	return partition;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BoidSet::Initialise(ResourceCache* pRes, Scene* pScene, std::string n)
{
	for (int i = 0; i < NumBoids; i++)
	{
		boidList[i].Initialise(pRes, pScene, n);
		boidList[i].SetID(i);
	}
	SetupBoidMap();
	UpdateBoidMap(true);
}

void BoidSet::SetupBoidMap()
{
	
	for (int x = 0; x < partitionsX; x++)
	{
		std::vector<std::vector<std::vector<Boid>>> a;
		for (int y = 0; y < partitionsY; y++)
		{
			std::vector<std::vector<Boid>> b;
			for (int z = 0; z < partitionsZ; z++)
			{
				std::vector<Boid> empty;
				b.push_back(empty);
			}
			a.push_back(b);
		}
		boidWorld.push_back(a);
	}
	std::cout << boidWorld.size() << " " << partitionsX << "\n" << boidWorld[0].size() << " " << partitionsY << "\n" << boidWorld[0][0].size() << " " << partitionsZ << "\n";
}

void BoidSet::UpdateBoidMap(bool startup)
{
	for (int i = 0; i < NumBoids; i++)
	{
		Vector3 oldPartition = boidList[i].GetPartition();
		boidList[i].UpdateCurrentPartition();
		Vector3 newPartition = boidList[i].GetPartition();

		if (oldPartition == newPartition && !startup) continue; //if it's in the same partition don't move it

		//std::vector<Boid>::iterator iter = std::find(boidWorld[newPartition.x_][newPartition.y_][newPartition.z_].begin(), boidWorld[newPartition.x_][newPartition.y_][newPartition.z_].end(), boidList[i]);
		//int index = std::distance(boidWorld[newPartition.x_][newPartition.y_][newPartition.z_].begin(), iter);
		int index = -1;
		int s = boidWorld[oldPartition.x_][oldPartition.y_][oldPartition.z_].size();
		//std::cout << "Start: " << s << "\n";
		if (s != 0)
		{
			for (int j = 0; j < s; j++)
			{
				if (boidWorld[oldPartition.x_][oldPartition.y_][oldPartition.z_][j].GetID() == boidList[i].GetID())
				{
					index = j;
				}
			}

			boidWorld[oldPartition.x_][oldPartition.y_][oldPartition.z_].erase(boidWorld[oldPartition.x_][oldPartition.y_][oldPartition.z_].begin() + index);//remove from old partition
		}

		

		auto it = boidWorld[newPartition.x_][newPartition.y_][newPartition.z_].insert(boidWorld[newPartition.x_][newPartition.y_][newPartition.z_].begin(),boidList[i]);//add at new partition
		//std::cout << "End: "<< boidWorld[newPartition.x_][newPartition.y_][newPartition.z_].size() << "\n";
	}
}

void BoidSet::Update(float tm, Projectile* proj)
{
	UpdateBoidMap();
	for (int i = 0; i < NumBoids; i++)
	{
		
		//Vector3 part = boidList[i].GetPartition();
		//update boidWorld vector

		//boidList[i].ComputeForce(&boidList[0], proj);

		//std::cout << part.x_ << " " << part.y_ << " " << part.x_ << "\n";

		boidList[i].ResetForce();
		boidList[i].ComputeForce(boidWorld, proj);
		/*
		for (int x = -1; x <= 1; x++)
		{
			for (int y = -1; y <= 1; y++)
			{
				for (int z = -1; z <= 1; z++)
				{
					if ((part.x_ + x >= 0 && part.x_ + x <= partitionsX - 1) && (part.y_ + y >= 0 && part.y_ + y <= partitionsY - 1) && (part.z_ + z >= 0 && part.z_ + z <= partitionsZ - 1))
					{
						boidList[i].ComputeForce(boidWorld[part.x_ + x][part.y_ + y][part.z_ + z], boidWorld[part.x_ +x][part.y_ + y][part.z_ + z].size(), proj);
					}
				}
			}
		}
		*/
		boidList[i].Update(tm, proj);
	}
	
}
