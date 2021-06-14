#pragma once

#include <Urho3D/Engine/Application.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Controls.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include<vector>
#include<exception>
#include <string>

#include "projectile.h"

namespace Urho3D
{
	class Node;
	class Scene;
	class RigidBody;
	class CollisionShape;
	class ResourceCache;
}
//All Urho3D classes reside in namespace Urho3D
using namespace Urho3D;

int const static NumBoids = 300;
float const static worldBounds[3][2] = { {-400.0f, 400.0f}, {0.0f, 100.0f}, {-400.0f, 400.0f} };	 //x,y,z       lower,upper

int const static partitionSize = 25; //partitions should be the same size in all axis, also allows for better scaling if world size increases 
int const static partitionsX = (int)ceil((worldBounds[0][1] - worldBounds[0][0]) / partitionSize)+1;
int const static partitionsY = (int)ceil((worldBounds[1][1] - worldBounds[1][0]) / partitionSize)+1;
int const static partitionsZ = (int)ceil((worldBounds[2][1] - worldBounds[2][0]) / partitionSize)+1;



class Boid
{
	static float Range_FAttract; 
	static float Range_FRepel; 
	static float Range_FAlign;
	static float FAttract_Factor; 
	static float FRepel_Factor; 
	static float FAlign_Factor; 
	static float FAttract_Vmax;

	static float minSpeed;
	static float maxSpeed;


	public:
		///Constructor
		Boid();
		//Destructor
		~Boid();

		void Initialise(ResourceCache* pRCache, Scene* pScene, std::string n);

		void ComputeForce(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi, Projectile* proj);

		void ResetForce();

		void Update(float time, Projectile* projs);

		void UpdateCurrentPartition();

		Vector3 GetPartition();

		void SetID(int ID);
		int GetID();
		bool GetEnabled();

	private:
		int id;
		std::string name;
		Vector3 force;
		Node* pNode;
		RigidBody* pRigidBody;
		CollisionShape* pCollisionShape;
		StaticModel* pObject;
		Vector3 partition;

		void Attract(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi);
		void Repel(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi);
		void Repel(Projectile* proj);
		void Align(std::vector<std::vector<std::vector<std::vector<Boid>>>> & boi);

		


};


class BoidSet
{
	public:
		
		Boid boidList[NumBoids];
		std::vector<std::vector<std::vector<std::vector<Boid>>>> boidWorld;//[partitionsX][partitionsY][partitionsZ][boids in the partition];
		//Boid[,,] boidWorld;
		
		BoidSet() {};
		void Initialise(ResourceCache* pRes, Scene* pScene, std::string n);
		void Update(float tm, Projectile* proj);
		void UpdateBoidMap(bool startup=false);
		void SetupBoidMap();
};