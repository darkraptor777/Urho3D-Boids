//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/CoreEvents.h>
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
#include <Urho3D/IO/Log.h>

#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/PhysicsEvents.h>

#include <Urho3D/Resource/ResourceCache.h>

#include <Urho3D/Scene/Scene.h>

#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include<Urho3D/UI/LineEdit.h>
#include<Urho3D/UI/Button.h>
#include<Urho3D/UI/UIEvents.h>
#include<Urho3D/UI/Window.h>
#include<Urho3D/UI/CheckBox.h>

#include <Urho3D/Network/Connection.h>
#include <Urho3D/Network/Network.h>
#include <Urho3D/Network/NetworkEvents.h>

#include <Urho3D/DebugNew.h>

//NOTE: if want to transfer stuff like mushroom count from server to client would want to create nonlocal object that contains this variable
//(Basically create non-local config object for this (however in general not much need for this unless trying to enforce some kind of anti-cheat))

#include "Character.h"
#include "CharacterDemo.h"
#include "Touch.h"
#include "boids.h"
#include "projectile.h"

//Custom Events
static const StringHash E_CLIENTCUSTOMEVENTBYLUKE("ClientCustomEventByLuke");

//Custom remote event used to tell the client which object them control
static const StringHash E_CLIENTOBJECTAUTHORITY("ClientObjectAuthority");
//Identifier for the node ID parameter in the event data
static const StringHash PLAYER_ID("Identity");
//Custom event on server, client has pressed button that is wants to start game
static const StringHash E_CLIENTISREADY("ClientReadyToStart");


URHO3D_DEFINE_APPLICATION_MAIN(CharacterDemo)

CharacterDemo::CharacterDemo(Context* context) :
	Sample(context),
	firstPerson_(false)
{
	//TUTORIAL: TODO

}

CharacterDemo::~CharacterDemo()
{
}

void CharacterDemo::Start()
{
	OpenConsoleWindow();
	Log::WriteRaw("Console Opened");
	// Execute base class startup
	Sample::Start();
	if (touchEnabled_)
		touch_ = new Touch(context_, TOUCH_SENSITIVITY);
	Log::WriteRaw("Client Startup");
	//TUTORIAL: TODO
	CreateScene();
	Log::WriteRaw("Scene Created");
	//Subscribe to necessary events
	SubscribeToEvents();

	//Set the mouse mode to use in the sample
	Sample::InitMouseMode(MM_RELATIVE);

	CreateMainMenu();
}

void CharacterDemo::CreateScene()
{
	//TUTORIAL: TODO
	//allows accessing resources
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	scene_ = new Scene(context_);
	//Create scene subsystem components
	scene_->CreateComponent<Octree>(LOCAL);
	scene_->CreateComponent<PhysicsWorld>(LOCAL);


	//Camera node Creation
	cameraNode_ = new Node(context_);
	Camera* camera = cameraNode_->CreateComponent<Camera>(LOCAL);
	cameraNode_->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
	camera->SetFarClip(300.0f);

	//Viewport configration
	GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));

	//Create static scene content. First create a zone for ambient lighting and fog control
	Node* zoneNode = scene_->CreateChild("Zone", LOCAL);
	Zone* zone = zoneNode->CreateComponent<Zone>();
	zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
	zone->SetFogColor(Color(0.1f, 0.1f, 0.5f));
	zone->SetFogStart(100.0f);
	zone->SetFogEnd(300.0f);
	zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));

	//Create a directional light with cascaded shadow mappign
	Node* lightNode = scene_->CreateChild("DirectionalLight", LOCAL);
	lightNode->SetDirection(Vector3(0.3f, -0.5f, 0.425f));
	Light* light = lightNode->CreateComponent<Light>();
	light->SetLightType(LIGHT_DIRECTIONAL);
	light->SetCastShadows(true);
	light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
	light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
	light->SetSpecularIntensity(0.5f);

	//Create the floor object
	Node* floorNode = scene_->CreateChild("Floor", LOCAL);
	floorNode->SetPosition(Vector3(0.0f, -0.5f, 0.0f));
	floorNode->SetScale(Vector3(800.0f, 1.0f, 800.0f));
	StaticModel* object = floorNode->CreateComponent<StaticModel>();
	object->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
	object->SetMaterial(cache->GetResource<Material>("Materials/Sand.xml"));

	RigidBody* body = floorNode->CreateComponent<RigidBody>();
	//Use collision layer bit 2 to mark world scenery. This is what we will raycast against to prevent camera from going inside geometry
	body->SetCollisionLayer(2);
	CollisionShape* shape = floorNode->CreateComponent<CollisionShape>();
	shape->SetBox(Vector3::ONE);

	//Mushroom Maker
	const unsigned NUM_MUSHROOMS = 50;
	for (unsigned i = 0; i < NUM_MUSHROOMS; ++i)
	{
		Node* objectNode = scene_->CreateChild("Mushroom", LOCAL);
		objectNode->SetPosition(Vector3(Random(400.0f) - 200.0f, 0.0f, Random(400.0f) - 200.0f));
		objectNode->SetRotation(Quaternion(0.0f, Random(360.0f), 0.0f));
		objectNode->SetScale(1.0f + Random(5.0f));

		StaticModel* object = objectNode->CreateComponent<StaticModel>();
		object->SetModel(cache->GetResource<Model>("Models/rib.mdl"));
		object->SetMaterial(cache->GetResource<Material>("Materials/rib.xml"));
		object->SetCastShadows(true);

		RigidBody* body = objectNode->CreateComponent<RigidBody>();
		body->SetCollisionLayer(2);
		CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
		shape->SetTriangleMesh(object->GetModel(), 0);
		//shape->SetBox(Vector3::ONE);
	}


	//Boxer
	const unsigned NUM_BOXES = 50;
	for (unsigned i = 0; i < NUM_BOXES; ++i)
	{
		float scale = Random(1.5f) + 0.5f;

		Node* objectNode = scene_->CreateChild("Box"); //boxes are dynamic and not static
		objectNode->SetPosition(Vector3(Random(400.0f) - 200.0f, Random(10.0f) + 10.0f, Random(400.0f) - 200.0f));
		objectNode->SetRotation(Quaternion(Random(360.0f), Random(360.0f), Random(360.0f)));
		objectNode->SetScale(scale);

		StaticModel* object = objectNode->CreateComponent<StaticModel>();
		object->SetModel(cache->GetResource<Model>("Models/shell.mdl"));
		object->SetMaterial(cache->GetResource<Material>("Materials/shell.xml"));
		object->SetCastShadows(true);

		RigidBody* body = objectNode->CreateComponent<RigidBody>();
		body->SetCollisionLayer(2);
		//Bigger boxes are heavier and harder to move
		body->SetMass(scale * 4.0f);
		CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
		shape->SetBox(Vector3::ONE);

	}


	plesiosaurs = new BoidSet();
	plesiosaurs->Initialise(cache, scene_, "plesiosaur");
	ichthyosaurs = new BoidSet();
	ichthyosaurs->Initialise(cache, scene_, "ichthyosaur");

	missile = new Projectile();
	missile->Initialise(cache, scene_, cameraNode_);
}

void CharacterDemo::CreateCharacter()
{
	//TUTORIAL: TODO
}

void CharacterDemo::CreateInstructions()
{
	//TUTORIAL: TODO
}

void CharacterDemo::SubscribeToEvents()
{
	
	//Subscribe to Update event for setting the character controls
	SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(CharacterDemo, HandleUpdate));

	SubscribeToEvent(E_CLIENTCONNECTED, URHO3D_HANDLER(CharacterDemo, HandleClientConnected));
	SubscribeToEvent(E_CLIENTDISCONNECTED, URHO3D_HANDLER(CharacterDemo, HandleClientDisconnected));

	SubscribeToEvent(E_CLIENTDISCONNECTED, URHO3D_HANDLER(CharacterDemo, HandleClientFinishedLoading));

	SubscribeToEvent(E_PHYSICSPRESTEP, URHO3D_HANDLER(CharacterDemo, HandlePhysicsPreStep));

	SubscribeToEvent(E_CLIENTCUSTOMEVENTBYLUKE, URHO3D_HANDLER(CharacterDemo, HandleCustomEventByLuke)); //subscribe to custom event
	GetSubsystem<Network>()->RegisterRemoteEvent(E_CLIENTCUSTOMEVENTBYLUKE); //authorise custom event

	SubscribeToEvent(E_CLIENTISREADY, URHO3D_HANDLER(CharacterDemo, HandleClientToServerReadyToStart));
	GetSubsystem<Network>()->RegisterRemoteEvent(E_CLIENTISREADY);

	SubscribeToEvent(E_CLIENTOBJECTAUTHORITY, URHO3D_HANDLER(CharacterDemo, HandleServerToClientObjectID));
	GetSubsystem<Network>()->RegisterRemoteEvent(E_CLIENTOBJECTAUTHORITY);

}

void CharacterDemo::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
	
	//TUTORIAL: TODO
	using namespace Update;
	//Take the time frame step, stored as a float
	float timestep = eventData[P_TIMESTEP].GetFloat();

	//Do not move if the UI has a focused element (the console)
	

	Input* input = GetSubsystem<Input>();

	//Movement speed measured in world units per second
	const float MOVE_SPEED = 15.0f;
	//Mouse sensitivity as degrees per pixel
	const float MOUSE_SENSITIVITY = 0.05f;

	//Use this frame's mousee motion to adjust camera node yaw and pitch.
	//Clamp the pitch between -90 and 90 degrees
	if (!menuVisible)
	{
		IntVector2 mouseMove = input->GetMouseMove();
		yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
		pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
		pitch_ = Clamp(pitch_, -90.0f, 90.0f);

		//Construct new orientation for the camera scene node from
		//yaw and pitch. Roll is fixed to zero
		cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
	}
	//Read WASD keys and move the camera scene node in the corresponding
	//direction if the are pressed, use the Translate() function
	//(default local space) to move relative to the node's orientation.
	if (input->GetKeyDown(KEY_W))
		cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timestep);
	if (input->GetKeyDown(KEY_S))
		cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timestep);
	if (input->GetKeyDown(KEY_A))
		cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timestep);
	if (input->GetKeyDown(KEY_D))
		cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timestep);
	if (input->GetKeyDown(KEY_F))
		missile->Fire();
	if (input->GetKeyPress(KEY_M))
		menuVisible = !menuVisible;


	missile->Update(timestep);
	plesiosaurs->Update(timestep, missile);
	ichthyosaurs->Update(timestep, missile);

	HandlePostUpdate(eventType, eventData);
}

void CharacterDemo::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
	//TUTORIAL: TODO
	// menu visible & invisible
	UI* ui = GetSubsystem<UI>();
	Input* input = GetSubsystem<Input>();
	ui->GetCursor()->SetVisible(menuVisible);
	window_->SetVisible(menuVisible);
}

void CharacterDemo::CreateMainMenu()
{
	// Set the mouse mode to use in the sample
	InitMouseMode(MM_RELATIVE);

	ResourceCache* cache = GetSubsystem<ResourceCache>();
	UI* ui = GetSubsystem<UI>();
	UIElement* root = ui->GetRoot();
	XMLFile* uiStyle = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
	root->SetDefaultStyle(uiStyle);  //need to set default ui style

	//Create a Cursor UI element.We want to be able to hide and show the main menu at will. When hidden, the mouse will control the camera, and when visible, the mouse will be able to interact with the GUI.

	SharedPtr<Cursor> cursor(new Cursor(context_));
	
	cursor->SetStyleAuto(uiStyle);
	ui->SetCursor(cursor);
	ui->GetCursor()->SetPosition(HA_CENTER, VA_CENTER);

	// Create the Window and add it to the UI's root node
	window_ = new Window(context_);
	root->AddChild(window_);

	// Set Window size and layout settings
	window_->SetMinWidth(384);
	window_->SetLayout(LM_VERTICAL, 6, IntRect(6, 6, 6, 6));
	window_->SetAlignment(HA_CENTER, VA_CENTER);
	window_->SetName("Window");
	window_->SetStyleAuto();

	connectButton_ = CreateButton("Connect", 24, window_);
	window_->AddChild(connectButton_);
	serverAddressLineEdit_ = CreateLineEdit("IP Address", 24, window_);
	window_->AddChild(serverAddressLineEdit_);
	disconnectButton_ = CreateButton("Disconnect", 24, window_);
	window_->AddChild(disconnectButton_);
	startServerButton_ = CreateButton("Start Server", 24, window_);
	window_->AddChild(startServerButton_);
	startClientGameButton_ = CreateButton("Client: Start Game", 24, window_);
	window_->AddChild(startClientGameButton_);
	quitButton_ = CreateButton("Quit", 36, window_);
	window_->AddChild(quitButton_);

	SubscribeToEvent(connectButton_, E_RELEASED, URHO3D_HANDLER(CharacterDemo, HandleConnect));
	SubscribeToEvent(quitButton_, E_RELEASED, URHO3D_HANDLER(CharacterDemo,HandleQuit));		//object to watch, event to watch for, what to do if event happens
	SubscribeToEvent(startServerButton_, E_RELEASED, URHO3D_HANDLER(CharacterDemo, HandleStartServer));
	SubscribeToEvent(startClientGameButton_, E_RELEASED, URHO3D_HANDLER(CharacterDemo, HandleClientStartGame));
}


Button* CharacterDemo::CreateButton(const String& text, int pHeight, Urho3D::Window* whichWindow)
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	Font* font = cache->GetResource<Font>("Fonts/Anonymous Pro.ttf");
	Button* button = window_->CreateChild<Button>();
	button->SetMinHeight(pHeight);
	button->SetStyleAuto();
	Text* buttonText = button->CreateChild<Text>();
	buttonText->SetFont(font, 12);
	buttonText->SetAlignment(HA_CENTER, VA_CENTER);
	buttonText->SetText(text);
	
	return button;
}

LineEdit* CharacterDemo::CreateLineEdit(const String& text, int pHeight, Urho3D::Window* whichWindow)
{
	LineEdit* lineEdit = window_->CreateChild<LineEdit>(); //input text box
	lineEdit->SetMinHeight(pHeight);
	lineEdit->SetAlignment(HA_CENTER, VA_CENTER);
	lineEdit->SetText(text);
	lineEdit->SetStyleAuto();

	return lineEdit;
}

void CharacterDemo::HandleConnect(StringHash eventType, VariantMap& eventData)
{
	Network* network = GetSubsystem<Network>();
	String address = serverAddressLineEdit_->GetText().Trimmed();
	if (address.Empty())
		address = "localhost"; // Use localhost to connect if nothing else specified

	// Connect to server, specify scene to use as a client for replication
	clientObjectID_ = 0; // Reset own object ID from possible previous connection
	network->Connect(address, SERVER_PORT, scene_);

	//UpdateButtons();
}

void CharacterDemo::HandleQuit(StringHash eventType, VariantMap& eventData)
{
	engine_->Exit();
}


void CharacterDemo::HandleStartServer(StringHash eventType, VariantMap& eventData)
{
	Log::WriteRaw("(HandleStartServer called) Server is started!");
	menuVisible = !menuVisible;
	Network* network = GetSubsystem<Network>();
	network->StartServer(SERVER_PORT);
	
}

void CharacterDemo::HandleClientConnected(StringHash eventType, VariantMap& eventData)
{
	Log::WriteRaw("(HandleClientConnected called) A client has connected!");
	using namespace ClientConnected;

	Connection* newConnection = static_cast<Connection*>(eventData[P_CONNECTION].GetPtr());
	newConnection->SetScene(scene_);

	//send an event to the client that has just connected
	VariantMap remoteEventData;
	remoteEventData["aValueRemoteValue"] = 0;
	newConnection->SendRemoteEvent(E_CLIENTCUSTOMEVENTBYLUKE, true, remoteEventData);
	//or send to all clients:
	//network->BroadcastRemoteEvent(E_CLIENTCUSTOMEVENTBYLUKE, true, remoteEventData);
}
void CharacterDemo::HandleClientDisconnected(StringHash eventType, VariantMap& eventData)
{
	Log::WriteRaw("(HandleClientDisconnected called) A client has disconnected!");

	Network* network = GetSubsystem<Network>();
	Connection* serverConnection = network->GetServerConnection();

	//Running as client
	if (serverConnection)
	{
		serverConnection->Disconnect();
		scene_->Clear(true, false);
		clientObjectID_ = 0;
	}
	//Running as a server, stop it
	else if (network->IsServerRunning())
	{
		network->StopServer();
		scene_->Clear(true, false);
	}
}

void CharacterDemo::HandleClientStartGame(StringHash eventType, VariantMap& eventData)
{
	printf("Client has pressed START GAME\n");
	if (clientObjectID_ == 0) //client is still an observer
	{
		Network* network = GetSubsystem<Network>();
		Connection* serverConnection = network->GetServerConnection();
		if (serverConnection)
		{
			VariantMap remoteEventData;
			remoteEventData[PLAYER_ID] = 0;
			serverConnection->SendRemoteEvent(E_CLIENTISREADY, true, remoteEventData);
		}
	}
}

Node* CharacterDemo::CreateControllableObject()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	//Create the scene node & visual representation. This will be a replicated object.
	Node* ballNode = scene_->CreateChild("AClientBall");
	ballNode->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
	ballNode->SetScale(2.5f);
	StaticModel* ballObject = ballNode->CreateComponent<StaticModel>();
	ballObject->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
	ballObject->SetMaterial(cache->GetResource<Material>("Materials/StoneSmall.xml"));

	//Create the physics components
	RigidBody* body = ballNode->CreateComponent<RigidBody>();
	body->SetMass(1.0f);
	body->SetFriction(1.0f);
	//motion damping so that the ball can not accelerate limitlessly
	body->SetLinearDamping(0.25f);
	body->SetAngularDamping(0.25f);
	CollisionShape* shape = ballNode->CreateComponent<CollisionShape>();
	shape->SetSphere(1.0f);

	return ballNode;
}

void CharacterDemo::HandleServerToClientObjectID(StringHash eventType, VariantMap& eventData)
{
	clientObjectID_ = eventData[PLAYER_ID].GetUInt();
	printf("Client ID : %i \n", clientObjectID_);
}

void CharacterDemo::HandleClientToServerReadyToStart(StringHash eventType, VariantMap& eventData)
{
	printf("Event sent by the Client and running on Server: Client is ready to start the game\n");

	using namespace ClientConnected;
	Connection* newConnection = static_cast<Connection*>(eventData[P_CONNECTION].GetPtr());

	//Create a controllable object for that client
	Node* newObject = CreateControllableObject();
	serverObjects_[newConnection] = newObject;

	//Finally send the object's node ID using a remote event
	VariantMap remoteEventData;
	remoteEventData[PLAYER_ID] = newObject->GetID();
	newConnection->SendRemoteEvent(E_CLIENTOBJECTAUTHORITY, true, remoteEventData);
}

void CharacterDemo::HandlePhysicsPreStep(StringHash eventType, VariantMap& eventData)
{
	//Log::WriteRaw("(HandlePhysicsPreStep called)");
	Network* network = GetSubsystem<Network>();
	Connection* serverConnection = network->GetServerConnection();

	//Client: collect controls
	if (serverConnection)			//if connected to a server then must be client and not server
	{
		serverConnection->SetPosition(cameraNode_->GetPosition()); //send camera position
		serverConnection->SetControls(FromClientToServerControls()); //send controls

		//Tell server to call custom event onto server machine
		VariantMap remoteEventData;
		remoteEventData["aValueRemoteValue"] = 0;
		serverConnection->SendRemoteEvent(E_CLIENTCUSTOMEVENTBYLUKE, true, remoteEventData);
	}

	else if (network->IsServerRunning())
	{
		ProcessClientControls();
	}

}

Controls CharacterDemo::FromClientToServerControls()
{
	Input* input = GetSubsystem<Input>();
	Controls controls;

	//Check which button has been pressed, good idea to keep track
	controls.Set(CTRL_FORWARD, input->GetKeyDown(KEY_W));
	controls.Set(CTRL_BACK, input->GetKeyDown(KEY_S));
	controls.Set(CTRL_LEFT, input->GetKeyDown(KEY_A));
	controls.Set(CTRL_RIGHT, input->GetKeyDown(KEY_D));
	controls.Set(32, input->GetKeyDown(KEY_F));

	//mouse yaw
	controls.yaw_ = yaw_;

	return controls;
}

void CharacterDemo::ProcessClientControls()
{
	Network* network = GetSubsystem<Network>();
	const Vector<SharedPtr<Connection>>& connections = network->GetClientConnections();

	//cycle through each client
	for (unsigned i = 0; i < connections.Size(); ++i)
	{
		Connection* connection = connections[i];
		//Get the object this connection is controlling
		Node* ballNode = serverObjects_[connection];

		//Client has no item connected
		if (!ballNode) continue;

		RigidBody* body = ballNode->GetComponent<RigidBody>();
		//Get the last controls sent by the client
		const Controls& controls = connection->GetControls();
		//Torque is relative to the forward vector
		Quaternion rotation(0.0f, controls.yaw_, 0.0f);


		//const Controls& controls = connection->GetControls();
		const float MOVE_TORQUE = 5.0f;
		if (controls.buttons_ & CTRL_FORWARD)
		{
			printf("Received from Client: Controls buttons Forward\n");
			body->ApplyTorque(rotation * Vector3::RIGHT * MOVE_TORQUE);
		}
		if (controls.buttons_ & CTRL_BACK)
		{
			printf("Received from Client: Controls buttons Back\n");
			body->ApplyTorque(rotation * Vector3::LEFT * MOVE_TORQUE);
		}
		if (controls.buttons_ & CTRL_LEFT)
		{
			printf("Received from Client: Controls buttons Left\n");
			body->ApplyTorque(rotation * Vector3::FORWARD * MOVE_TORQUE);
		}
		if (controls.buttons_ & CTRL_RIGHT)
		{
			printf("Received from Client: Controls buttons Right\n");
			body->ApplyTorque(rotation * Vector3::BACK * MOVE_TORQUE);
		}
		if (controls.buttons_ & 32 )
		{
			printf("Received from Client: Controls buttons Fire\n");
			missile->Fire();
		}
	}

}

void CharacterDemo::HandleClientFinishedLoading(StringHash eventType, VariantMap& eventData)
{
	printf("Client has finished loading up the scene from the server\n");
}

void CharacterDemo::HandleCustomEventByLuke(StringHash eventType, VariantMap& eventData)
{
	int exampleValue = eventData["aValueRemoteValue"].GetUInt();
	printf("Custom Event triggered - value passed - %i \n", exampleValue);
}