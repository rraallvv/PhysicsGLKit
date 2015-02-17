//
//  GameViewController.m
//  BulletPhysics
//

#import "GameViewController.h"
#import <OpenGLES/ES2/glext.h>

//#define USE_TOKAMAK
#define USE_BULLET

#if defined(USE_TOKAMAK)
	#include "tokamak.h"
#elif defined(USE_BULLET)
	#include "btBulletDynamicsCommon.h"
#endif

#include "CubeVertexData.h"

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

// Uniform index.
enum
{
    UNIFORM_MODELVIEWPROJECTION_MATRIX,
    UNIFORM_NORMAL_MATRIX,
    NUM_UNIFORMS
};
GLint uniforms[NUM_UNIFORMS];

// Attribute index.
enum
{
    ATTRIB_VERTEX,
    ATTRIB_NORMAL,
    NUM_ATTRIBUTES
};

@interface GameViewController () {
	float _rotation;

    GLuint _vertexArray;
    GLuint _vertexBuffer;

#if defined(USE_TOKAMAK)

	neSimulator *_simulator;
	neRigidBody *_bodies[60];
	int _bodyCount;

#elif defined(USE_BULLET)

	btDiscreteDynamicsWorld *_discreteDynamicsWorld;
	btCollisionDispatcher *_collisionDispatcher;
	btDefaultCollisionConfiguration *_collisionConfig;
	btSequentialImpulseConstraintSolver *_constraintSolver;
	btDbvtBroadphase *_broadphase;

	btAlignedObjectArray<btCollisionShape*>  _collisionShapes;
	btAlignedObjectArray<btCollisionObject*> _collisionObjects;
	btAlignedObjectArray<btTypedConstraint*> _constraints;

#endif
}
@property (strong, nonatomic) EAGLContext *context;
@property (strong, nonatomic) GLKBaseEffect *effect;

- (void)setupGL;
- (void)tearDownGL;
@end

@implementation GameViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];

    if (!self.context) {
        NSLog(@"Failed to create ES context");
    }
    
    GLKView *view = (GLKView *)self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;

	self.preferredFramesPerSecond = 60;
    
    [self setupGL];
	[self setupPhysics];
}

- (void)dealloc
{    
	[self tearDownPhysics];
    [self tearDownGL];

    if ([EAGLContext currentContext] == self.context) {
        [EAGLContext setCurrentContext:nil];
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];

    if ([self isViewLoaded] && ([[self view] window] == nil)) {
        self.view = nil;
        
        [self tearDownGL];
        
        if ([EAGLContext currentContext] == self.context) {
            [EAGLContext setCurrentContext:nil];
        }
        self.context = nil;
    }

    // Dispose of any resources that can be recreated.
}

- (BOOL)prefersStatusBarHidden {
    return YES;
}

- (void)setupGL
{
    [EAGLContext setCurrentContext:self.context];

    self.effect = [[GLKBaseEffect alloc] init];
    self.effect.light0.enabled = GL_TRUE;
    self.effect.light0.diffuseColor = GLKVector4Make(0.64f, 0.09f, 0.09f, 1.0f);
	self.effect.light0.specularColor = GLKVector4Make(0.5f, 0.5f, 0.5f, 1.0f);
	self.effect.light0.position = GLKVector4Make(0.0f, 1.0f, 1.0f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    
    glGenVertexArraysOES(1, &_vertexArray);
    glBindVertexArrayOES(_vertexArray);
    
    glGenBuffers(1, &_vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gCubeVertexData), gCubeVertexData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 24, BUFFER_OFFSET(0));
    glEnableVertexAttribArray(GLKVertexAttribNormal);
    glVertexAttribPointer(GLKVertexAttribNormal, 3, GL_FLOAT, GL_FALSE, 24, BUFFER_OFFSET(12));
    
    glBindVertexArrayOES(0);
}

- (void)tearDownGL
{
    [EAGLContext setCurrentContext:self.context];
    
    glDeleteBuffers(1, &_vertexBuffer);
    glDeleteVertexArraysOES(1, &_vertexArray);
    
    self.effect = nil;
}

#pragma mark - Physics

- (void)setupPhysics
{
	// Create the discrete dynamic world

#if defined(USE_TOKAMAK)

	neSimulatorSizeInfo sizeInfo;


	sizeInfo.rigidBodiesCount = 100;
	sizeInfo.animatedBodiesCount = 0;
	s32 totalBody = sizeInfo.rigidBodiesCount + sizeInfo.animatedBodiesCount;
	sizeInfo.geometriesCount = totalBody;
	//see sdk for this formula:
	sizeInfo.overlappedPairsCount = totalBody * (totalBody - 1) / 2;
	sizeInfo.rigidParticleCount = 0;

	neV3 gravity;
	gravity.Set(0.0f, -9.8f, 0.0f);

	//all information gathered. So create the simulator:
	_simulator = neSimulator::CreateSimulator(sizeInfo, NULL, &gravity);

	float friction = 0.5f;
	float restitution = 0.0f;

	_simulator->SetMaterial(0, friction, restitution);

	_bodyCount = 0;

	// Add the physics bodies

	neV3 position;

	position.Set(-4.5f, 0.0f ,0.0f);
	[self addBoxesAtPosition:position];

	position.Set(4.5f, 0.0f, 0.0f);
	[self addBoxesAtPosition:position];

	position.Set(0.0f, 0.0f, 0.0f);
	[self addContainerAtPosition:position];

#elif defined(USE_BULLET)

	// Create the discrete dynamic world

	_broadphase = new btDbvtBroadphase();
	_constraintSolver = new btSequentialImpulseConstraintSolver;
	_collisionConfig = new btDefaultCollisionConfiguration();
	_collisionDispatcher = new btCollisionDispatcher(_collisionConfig);
	_discreteDynamicsWorld = new btDiscreteDynamicsWorld(_collisionDispatcher, _broadphase, _constraintSolver, _collisionConfig);

	_discreteDynamicsWorld->setGravity(btVector3(0.0f, -9.8f, 0.0f));


	// Add the physics bodies

	[self addBoxesAtPosition:btVector3(-4.5f, 0.0f ,0.0f)];

	[self addBoxesAtPosition:btVector3(4.5f, 0.0f, 0.0f)];

	[self addContainerAtPosition:btVector3(0.0f, 0.0f, 0.0f)];

#endif

}

- (void)tearDownPhysics
{

#if defined(USE_TOKAMAK)

	// Free the discrete dynamic world
	neSimulator::DestroySimulator(_simulator); //also cleans all bodies etc.

#elif defined(USE_BULLET)

	// Remove constraints
	for (int i=0;i<_constraints.size();i++) {
		btTypedConstraint* constraint = _constraints[i];
		_discreteDynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	// Remove collision objects
	for (int i=0;i<_collisionObjects.size();i++) {
		btCollisionObject* obj = _collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		_discreteDynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	// Remove collision shapes
	for (int i=0;i<_collisionShapes.size();i++) {
		btCollisionShape* shape = _collisionShapes[i];
		delete shape;
	}

	// Free the discrete dynamic world
	delete _discreteDynamicsWorld;
	delete _constraintSolver;
	delete _collisionDispatcher;
	delete _broadphase;
	delete _collisionConfig;

#endif
}

#if defined(USE_TOKAMAK)

- (void)addBoxesAtPosition:(neV3)position
{

	// Create an array of 3x3 boxes located at a given position

	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			for (int k=0; k<3; ++k)
			{
				// Create a rigid body using the CTokSim class:
				neRigidBody * body = _simulator->CreateRigidBody();

				body->SetMass(0.01f);

				// Add geometry to the body and set it to be a box of dimensions 1, 1, 1
				neGeometry *geom;
				geom = body->AddGeometry();
				neV3 boxSize;
				boxSize.Set(2.0f, 2.0f, 2.0f);
				geom->SetBoxSize(boxSize.X(), boxSize.Y(), boxSize.Z());
				body->SetInertiaTensor(neBoxInertiaTensor(boxSize.X(), boxSize.Y(), boxSize.Z(),body->GetMass()));

				// Update the bounding info of the object -- must always call this
				// after changing a body's geometry.
				body->UpdateBoundingInfo();

				neV3 origin;
				origin.Set(-1.0f, -1.0f, -1.0f);
				neV3 offset;

				offset.Set(float(i), float(j), float(k));

				neV3 pos = position + 3.0f * (origin + offset);
				body->SetPos(pos);

				_bodies[_bodyCount++] = body;
			}
		}
	}
}

- (void)addContainerAtPosition:(neV3)position
{

	// Add the container's rigid body

	neRigidBody *body = _simulator->CreateRigidBody();

	neGeometry *geom;
	neV3 pos;
	neT3 trans;


	geom = body->AddGeometry();
	geom->SetBoxSize(2.0f, 22.0f, 22.0f);

	pos.Set(-10.0f, 0.0f, 0.0f);

	trans.SetIdentity();
	trans.pos = pos;

	geom->SetTransform(trans);



	geom = body->AddGeometry();
	geom->SetBoxSize(2.0f, 22.0f, 22.0f);

	pos.Set(10.0f, 0.0f, 0.0f);

	trans.SetIdentity();
	trans.pos = pos;

	geom->SetTransform(trans);



	geom = body->AddGeometry();
	geom->SetBoxSize(22.0f, 2.0f, 22.0f);

	pos.Set(0.0f, -10.0f, 0.0f);

	trans.SetIdentity();
	trans.pos = pos;

	geom->SetTransform(trans);



	geom = body->AddGeometry();
	geom->SetBoxSize(22.0f, 2.0f, 22.0f);

	pos.Set(0.0f, 10.0f, 0.0f);

	trans.SetIdentity();
	trans.pos = pos;

	geom->SetTransform(trans);



	geom = body->AddGeometry();
	geom->SetBoxSize(22.0f, 22.0f, 2.0f);

	pos.Set(0.0f, 0.0f, -10.0f);

	trans.SetIdentity();
	trans.pos = pos;

	geom->SetTransform(trans);



	geom = body->AddGeometry();
	geom->SetBoxSize(22.0f, 22.0f, 2.0f);

	pos.Set(0.0f, 0.0f, 10.0f);

	trans.SetIdentity();
	trans.pos = pos;

	geom->SetTransform(trans);



	body->SetInertiaTensor(neBoxInertiaTensor(12.0f, 12.0f, 12.0f,body->GetMass()));

	body->UpdateBoundingInfo();
	body->SetMass(1.0f);
	pos.Set(0.0f,0.0f,0.0f);
	body->SetPos(pos);



	_bodies[_bodyCount++] = body;



	// Add the hinge constraint

	neJoint * joint;
	joint = _simulator->CreateJoint(body);

	trans.pos.Set(0.0f,0.0f,0.0f);
	trans.rot[0].Set(1.0f, 0.0f, 0.0f);
	trans.rot[1].Set(0.0f, 0.0f, 1.0f);
	trans.rot[2].Set(0.0f, 1.0f, 0.0f);
	joint->SetJointFrameWorld(trans);

	joint->SetType(neJoint::NE_JOINT_HINGE);
	joint->SetMotor(neJoint::NE_MOTOR_SPEED, -2.0f*M_PI/30.0f, 1024.0f);
	joint->EnableMotor(true);

	joint->Enable(true);
}

#elif defined(USE_BULLET)

- (void)addBoxesAtPosition:(btVector3)position
{
	// Create an array of 3x3 boxes located at a given position

	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			for (int k=0; k<3; ++k)
			{
				btScalar mass(0.01f);
				btVector3 localInertia(0.0f, 0.0f, 0.0f);

				btCollisionShape* shape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
				_collisionShapes.push_back(shape);
				shape->calculateLocalInertia(mass,localInertia);

				btTransform transform = btTransform::getIdentity();
				transform.setOrigin(position + btScalar(3.0f) * (btVector3(-1.0f, -1.0f, -1.0f) + btVector3(btScalar(i), btScalar(j), btScalar(k))));

				btDefaultMotionState *motionState = new btDefaultMotionState(transform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,shape,localInertia);

				btRigidBody* body = new btRigidBody(rbInfo);
				_collisionObjects.push_back(body);
				body->setRestitution(0.0f);

				_discreteDynamicsWorld->addRigidBody(body);
			}
		}
	}
}

- (void)addContainerAtPosition:(btVector3)position
{

	// Add the container's rigid body

	btCompoundShape *parentShape = new btCompoundShape;

	btTransform transform;
	transform.setIdentity();


	btCollisionShape* shape1 = new btBoxShape(btVector3(1.0f, 11.0f, 11.0f));
	_collisionShapes.push_back(shape1);

	transform.setOrigin(btVector3(-10.0f, 0.0f, 0.0f));
	parentShape->addChildShape(transform, shape1);

	transform.setOrigin(btVector3(10.0f, 0.0f, 0.0f));
	parentShape->addChildShape(transform, shape1);


	btCollisionShape* shape2 = new btBoxShape(btVector3(11.0f, 1.0f, 11.0f));
	_collisionShapes.push_back(shape2);

	transform.setOrigin(btVector3(0.0f, -10.0f, 0.0f));
	parentShape->addChildShape(transform, shape2);

	transform.setOrigin(btVector3(0,10,0));
	parentShape->addChildShape(transform, shape2);


	btCollisionShape* shape3 = new btBoxShape(btVector3(11.0f, 11.0f, 1.0f));
	_collisionShapes.push_back(shape3);

	transform.setOrigin(btVector3(0.0f, 0.0f, -10.0f));
	parentShape->addChildShape(transform, shape3);

	transform.setOrigin(btVector3(0.0f, 0.0f, 10.0f));
	parentShape->addChildShape(transform, shape3);


	btScalar mass(1.0f);
	btVector3 localInertia(0.0f, 0.0f, 0.0f);


	transform.setIdentity();
	transform.setOrigin(position);


	parentShape->calculateLocalInertia(mass,localInertia);
	btDefaultMotionState *motionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo2(mass,motionState,parentShape,localInertia);


	btRigidBody* body = new btRigidBody(rbInfo2);
	_collisionObjects.push_back(body);
	body->setRestitution(0.0f);


	_discreteDynamicsWorld->addRigidBody(body);


	// Add the hinge constraint

	btHingeConstraint* hinge = new btHingeConstraint(*body, btVector3(0.0f, 0.0f, 0.0f),  btVector3(0.0f, 0.0f, 1.0f));
	_constraints.push_back(hinge);

	hinge->enableAngularMotor(true, -4.0f*M_PI/60.0f, 1024.0f);

	_discreteDynamicsWorld->addConstraint(hinge);
	
}

#endif

#pragma mark - GLKView and GLKViewController delegate methods

- (void)update
{
	float aspect = fabsf(self.view.bounds.size.width / self.view.bounds.size.height);
	GLKMatrix4 projectionMatrix = GLKMatrix4MakePerspective(GLKMathDegreesToRadians(60.0f), aspect, 0.1f, 100.0f);

	self.effect.transform.projectionMatrix = projectionMatrix;

	static unsigned int step = 0;

	// The simulation is updated once every spreadSteps, by the coresponding amount of time elapsed since the last update
	const int spreadSteps = 1;

	if (step++ % spreadSteps == 0)
	{
		// Advance the physcis simulation
		float dt = spreadSteps * 1./60;

#if defined(USE_TOKAMAK)
		_simulator->Advance(dt);
#elif defined(USE_BULLET)
		_discreteDynamicsWorld->stepSimulation(dt, 0); // passing zero forces the simulation step to advance by exacty dt
#endif
	}
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClearColor(0.65f, 0.65f, 0.65f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glBindVertexArrayOES(_vertexArray);

	GLKMatrix4 baseModelViewMatrix = GLKMatrix4MakeTranslation(0.0f, 0.0f, -40.0f);


	// Update physics transformations

	GLKMatrix4 shapeScale;
	GLKMatrix4 modelViewMatrix;

#if defined(USE_TOKAMAK)

	for (int i=0; i<_bodyCount; ++i)
	{
		// For each RigidBody
		neRigidBody *body = _bodies[i];

		if (body)
		{
			neT3 bodyTrans = body->GetTransform();

			body->BeginIterateGeometry();

			for (int j=0; j<body->GetGeometryCount(); ++j)
			{
				if (j>3)
					continue;

				neGeometry * geom = body->GetNextGeometry();

				neV3 boxSize;
				geom->GetBoxSize(boxSize);
				neT3 boxTrans = geom->GetTransform();

				neT3 trans = bodyTrans*boxTrans;

				GLfloat m[] = {
					trans.rot[0][0], trans.rot[0][1], trans.rot[0][2], 0.0f,
					trans.rot[1][0], trans.rot[1][1], trans.rot[1][2], 0.0f,
					trans.rot[2][0], trans.rot[2][1], trans.rot[2][2], 0.0f,
					trans.pos[0],    trans.pos[1],    trans.pos[2],    1.0f
				};

				shapeScale = GLKMatrix4MakeScale(boxSize.X(), boxSize.Y(), boxSize.Z());

				modelViewMatrix = GLKMatrix4Multiply(GLKMatrix4MakeWithArray(m), shapeScale);

				modelViewMatrix = GLKMatrix4Multiply(baseModelViewMatrix, modelViewMatrix);

				self.effect.transform.modelviewMatrix = modelViewMatrix;
				[self.effect prepareToDraw];

				glDrawArrays(GL_TRIANGLES, 0, 36);
			}
		}
	}

#elif defined(USE_BULLET)

	// Update physics transformations

	ATTRIBUTE_ALIGNED16(float) m[16];
	btVector3 scaleSize;

	for (int i=0; i<_discreteDynamicsWorld->getNumCollisionObjects(); ++i)
	{
		// For each RigidBody
		btCollisionObject* obj = _discreteDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);

		if (body)
		{
			body->setActivationState(DISABLE_DEACTIVATION);

			btDefaultMotionState* motionState = (btDefaultMotionState*) body->getMotionState();

			btTransform transform;

			motionState->getWorldTransform(transform);

			btBoxShape *shape = (btBoxShape *)body->getCollisionShape();

			if (shape->isCompound())
			{
				btCompoundShape *compoundShape = (btCompoundShape *)shape;

				for (int j=0; j<compoundShape->getNumChildShapes()-2; ++j)
				{
					shape = (btBoxShape *)compoundShape->getChildShape(j);
					btTransform childTransform = compoundShape->getChildTransform(j);

					scaleSize = btScalar(2.0f)*shape->getHalfExtentsWithMargin();

					shapeScale = GLKMatrix4MakeScale(scaleSize.x(), scaleSize.y(), scaleSize.z());

					(transform*childTransform).getOpenGLMatrix(m);
					modelViewMatrix = GLKMatrix4Multiply(GLKMatrix4MakeWithArray(m), shapeScale);

					modelViewMatrix = GLKMatrix4Multiply(baseModelViewMatrix, modelViewMatrix);

					self.effect.transform.modelviewMatrix = modelViewMatrix;
					[self.effect prepareToDraw];

					glDrawArrays(GL_TRIANGLES, 0, 36);
				}

			} else {

				scaleSize = btScalar(2.0f)*shape->getHalfExtentsWithMargin();

				shapeScale = GLKMatrix4MakeScale(scaleSize.x(), scaleSize.y(), scaleSize.z());

				transform.getOpenGLMatrix(m);
				modelViewMatrix = GLKMatrix4Multiply(GLKMatrix4MakeWithArray(m), shapeScale);

				modelViewMatrix = GLKMatrix4Multiply(baseModelViewMatrix, modelViewMatrix);

				self.effect.transform.modelviewMatrix = modelViewMatrix;
				[self.effect prepareToDraw];
				
				glDrawArrays(GL_TRIANGLES, 0, 36);
			}
		}
	}

#endif
}

@end
