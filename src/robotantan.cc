// raylib+Bullet physics
// Sam Jackson
// Partially based off this: https://github.com/bulletphysics/bullet3/blob/master/examples/HelloWorld/HelloWorld.cpp

#include <vector>

#include <btBulletDynamicsCommon.h>
#include "raylib.h"

#if defined(PLATFORM_WEB)
    #include <emscripten/emscripten.h>
#endif

btDefaultCollisionConfiguration* collision_configuration;
btCollisionDispatcher* dispatcher; // Collision dispatcher, handles collision
btBroadphaseInterface* overlapping_pair_cache; // Broadphase interface, detects overlapping objects
btSequentialImpulseConstraintSolver* solver; // Constraint solver
btDiscreteDynamicsWorld* dynamics_world; // The world where physics takes place
btAlignedObjectArray<btCollisionShape*> collision_shapes; // Keeps track of collision shapes

// Collider shape
enum Shape {
	CUBE,
	SPHERE,
};

enum PhysicsType {
	STATIC,
	DYNAMIC,
};

class PhysicsObject {
private:
	btRigidBody* body;
	btCollisionShape* collider_shape;
	Model model;

public:
	Color color;

	PhysicsObject(
		Vector3 position = {0,0,0},
		Vector3 rotation = {0,0,0},
		Vector3 size = {1,1,1},
		Shape shape = CUBE,
		PhysicsType type = STATIC,
		float mass = 0,
		Color color = WHITE
	) {
		this->color = color;

		if (shape == CUBE) {
			collider_shape = new btBoxShape(
				btVector3( btScalar(size.x/2.0), btScalar(size.y/2.0), btScalar(size.z/2.0) )
			);

			model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));
		} else if (shape == SPHERE) {
			collider_shape = new btSphereShape(btScalar(size.x));
			model = LoadModelFromMesh(GenMeshSphere(size.x, 8, 16));
		}

		collision_shapes.push_back(collider_shape);

		// Set location and rotation
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin( btVector3(position.x, position.y, position.z) );
		transform.setRotation(
			btQuaternion( btScalar(rotation.z), btScalar(rotation.y), btScalar(rotation.x) )
		);

		btScalar object_mass(mass); // Set the object's mass

		// Calculate local inertia for dynamic objects
		btVector3 local_inertia(0, 0, 0);
		if (type == DYNAMIC || mass != 0.0) // Objects with 0.0 mass are static
			collider_shape->calculateLocalInertia(mass, local_inertia);

		btDefaultMotionState* motion_state = new btDefaultMotionState(transform);

		btRigidBody::btRigidBodyConstructionInfo rb_info(object_mass, motion_state, collider_shape, local_inertia);
		body = new btRigidBody(rb_info);

		dynamics_world->addRigidBody(body); // Add the body to the dynamics world
	}

	void render() {
		const float radian_scale = 57.296;
		btTransform trans;

		// Get the transform of the body
		if (body && body->getMotionState())
			body->getMotionState()->getWorldTransform(trans);
		else
			return;


		// Get position from transform
		Vector3 position = {
			float(trans.getOrigin().getX()),
			float(trans.getOrigin().getY()),
			float(trans.getOrigin().getZ())
		};

		// Get rotation from transform
		btQuaternion quat = trans.getRotation();

		Vector3 axis = {
			float(quat.getAxis().getX()),
			float(quat.getAxis().getY()),
			float(quat.getAxis().getZ())
		};
		float angle = float( quat.getAngle() ) * radian_scale; // Convert radians to degrees

		// Render model
		DrawModelEx(model, position, axis, angle, {1,1,1}, color);
		DrawModelWiresEx(
			model, position, axis, angle, {1,1,1},
			{(unsigned char)(color.r/2), (unsigned char)(color.g/2), (unsigned char)(color.b/2), color.a}
		);
	}

	void unload() {
		UnloadModel(model);
	}
};

void UpdateDrawFrame(void);     // Update and Draw one frame


std::vector<PhysicsObject> physics_objects;
Camera3D camera = { 0 };
const int FPS = 45;

int main(int argc, char** argv) {
	const int screen_width = 800;
	const int screen_height = 450;

	SetConfigFlags(FLAG_MSAA_4X_HINT); // Enable anti-aliasing
	InitWindow(screen_width, screen_height, "raylib + Bullet");

	// Initialize Bullet
	collision_configuration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collision_configuration);
	overlapping_pair_cache = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	dynamics_world = new btDiscreteDynamicsWorld(dispatcher, overlapping_pair_cache, solver, collision_configuration);

	dynamics_world->setGravity(btVector3(0, -10, 0));

	physics_objects = {
		PhysicsObject( {0,0,0}, {0,0,0}, {8,0.5,8}, CUBE, STATIC, 0.0, GRAY ) // Floor
	};

	// Set up the camera
	camera.position = (Vector3){ 10.0f, 5.0f, 10.0f };
	camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 60.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	SetTargetFPS(FPS);

	// Create a group of random dynamic objects
	for (size_t i = 0; i < 30; i++) {
		const std::vector<Color> colors {
			MAROON, ORANGE, DARKGREEN, DARKBLUE, DARKPURPLE,
			RED, GOLD, LIME, BLUE, VIOLET, PINK, YELLOW,
			GREEN, SKYBLUE, PURPLE
		};

		physics_objects.push_back(
			PhysicsObject(
				{(float)GetRandomValue(-5, 5), (float)GetRandomValue(10, 15), (float)GetRandomValue(-5, 5)},
				{(float)GetRandomValue(-3, 3), (float)GetRandomValue(-3, 3), (float)GetRandomValue(-3, 3)},
				{(float)GetRandomValue(1, 3), (float)GetRandomValue(1, 3), (float)GetRandomValue(1, 3)},
				(Shape)GetRandomValue(0, 1),
				DYNAMIC,
				1.0,
				colors[ GetRandomValue(0, colors.size() - 1) ]
			)
		);
	}


#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
  // Main game loop
	while (!WindowShouldClose()) {
		UpdateDrawFrame();
	}

	// Cleanup
	// Remove rigid bodies
	for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
		btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		dynamics_world->removeCollisionObject(obj);
		delete obj;
	}

	// Delete collision shapes
	for (int i = 0; i < collision_shapes.size(); i++) {
		btCollisionShape* shape = collision_shapes[i];
		collision_shapes[i] = 0;
		delete shape;
	}

	// Unload physics objects
	for (auto& object : physics_objects) {
		object.unload();
	}

	delete dynamics_world;
	delete solver;
	delete overlapping_pair_cache;
	delete dispatcher;
	delete collision_configuration;

	CloseWindow();
#endif
}

void UpdateDrawFrame(void)
{
		UpdateCamera(&camera, CAMERA_ORBITAL);

		dynamics_world->stepSimulation(1.0 / float(FPS), 10); // Update physics

		BeginDrawing();

			ClearBackground(RAYWHITE);

			BeginMode3D(camera);

				for (auto& object : physics_objects) {
					object.render();
				}

				DrawGrid(10, 1.0);

			EndMode3D();

			DrawFPS(16, 16);

		EndDrawing();
}