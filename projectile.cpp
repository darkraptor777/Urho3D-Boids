#include "projectile.h"

float Projectile::speed = 200.0f;
float Projectile::lifespan = 2.0f;

///Constructor
Projectile::Projectile()
{
	pNode = nullptr;
	pRigidBody = nullptr;
	pCollisionShape = nullptr;
	pObject = nullptr;
	active = false;
	aliveFor = 0.0f;
}
//Destructor
Projectile::~Projectile()
{
}



void Projectile::Initialise(ResourceCache* pRCache, Scene* pScene, Node* camera)
{
	cam = camera;

	pNode = pScene->CreateChild("Projectile");
	pNode->SetPosition(cam->GetPosition());
	pNode->SetRotation(cam->GetRotation());
	pNode->SetScale(2.0f);

	pObject = pNode->CreateComponent<StaticModel>();
	pObject->SetModel(pRCache->GetResource<Model>("Models/Sphere.mdl"));
	pObject->SetMaterial(pRCache->GetResource<Material>("Materials/Water.xml"));
	pObject->SetCastShadows(true);

	pRigidBody = pNode->CreateComponent<RigidBody>();
	pRigidBody->SetCollisionLayer(2);
	
	pRigidBody->SetMass(1.0f);
	pRigidBody->SetUseGravity(false);
	pRigidBody->SetPosition(cam->GetPosition());
	pRigidBody->SetLinearVelocity(Vector3(Random(40.0f) - 20.0f, 0.0f, Random(40.0f) - 20.0f));

	pCollisionShape = pNode->CreateComponent<CollisionShape>();
	pCollisionShape->SetTriangleMesh(pObject->GetModel(), 0);

	active = false;
	pObject->SetEnabled(false);
}

void Projectile::Update(float time)
{
	if (active)
	{
		aliveFor += time;
		if (lifespan < aliveFor)
		{
			active = false;
			pObject->SetEnabled(false);
		}
	}
}

void Projectile::Fire()
{
	if (active)
		return;

	active = true;
	pObject->SetEnabled(true);

	aliveFor = 0.0f;
	pRigidBody->SetPosition(cam->GetPosition());
	pRigidBody->SetLinearVelocity(cam->GetDirection().Normalized() * speed);
}

RigidBody* Projectile::getRB()
{
	return pRigidBody;
}