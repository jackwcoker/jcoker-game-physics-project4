/// \file Game.cpp
/// \brief Code for the game class CGame.

#include "Game.h"

#include "GameDefines.h"
#include "Renderer.h"
#include "ComponentIncludes.h"
#include "ObjectManager.h"
#include "Descriptors.h"

#include "shellapi.h"

/// \brief Callback function for the end of a physics tick.
///
/// This function gets called by Bullet Physics at the end of a physics tick,
/// which may be more than once a frame. We use it to notify objects of 
/// contacts with other objects. The objects are then responsible for playing
/// their collision sounds. The user pointers of the two physics bodies that
/// are in contact are assumed to point to two corresponding `CObject`s. 
/// \param p Pointer to Physics World.
/// \param t Time step (unused).

void myTickCallback(btDynamicsWorld *p, btScalar t){
  UNREFERENCED_PARAMETER(t);

  const UINT nManifolds = (UINT)p->getDispatcher()->getNumManifolds();

  for(UINT i=0; i<nManifolds; i++){ //for each manifold
    btDispatcher* pDispatcher = p->getDispatcher(); //pointer to dispatcher
    btPersistentManifold* pManifold = pDispatcher->getManifoldByIndexInternal(i);

    //get body pointers from manifold
    const btRigidBody* pBody0 = btRigidBody::upcast(pManifold->getBody0()); 
    const btRigidBody* pBody1 = btRigidBody::upcast(pManifold->getBody1()); 
    
    //get object pointers from body pointers
    CObject* pObj0 = (CObject*)pBody0->getUserPointer();
    CObject* pObj1 = (CObject*)pBody1->getUserPointer(); 
    
    const int nContacts = pManifold->getNumContacts(); //number of contact points

    if(pObj0 && pObj1 && nContacts > 0){ //guard
      CContactDesc d(nContacts); //contact descriptor

      //compute impulse from contact points

      for(int j=0; j<nContacts; j++){ 
        btManifoldPoint& pt = pManifold->getContactPoint(j);
        d.m_fImpulse = std::max(d.m_fImpulse, pt.getAppliedImpulse());
      } //for

      pObj0->AddContact(pObj1, d); //add contact to one object only
    } //if
  } //for
} //myTickCallback

////////////////////////////////////////////////////////////////////////////////
// CGame functions.

/// Destructor.

CGame::~CGame(){
  delete m_pObjectManager;

  //delete collision shapes
  for(int j=0; j<m_btCollisionShapes.size(); j++){
    btCollisionShape* shape = m_btCollisionShapes[j];
    m_btCollisionShapes[j] = 0;
    delete shape;
  } //for
  
  delete m_pPhysicsWorld;
  delete m_pSolver;
  delete m_pBroadphase;
  delete m_pDispatcher;
  delete m_pConfig;
} //destructor

/// Initialize the Bullet Physics engine.

void CGame::InitBulletPhysics(){
  m_pConfig     = new btDefaultCollisionConfiguration();
  m_pDispatcher = new btCollisionDispatcher(m_pConfig);
  m_pBroadphase = new btDbvtBroadphase();
  m_pSolver     = new btSequentialImpulseConstraintSolver;

  m_pPhysicsWorld = new btDiscreteDynamicsWorld(
    m_pDispatcher, m_pBroadphase, m_pSolver, m_pConfig);

  m_pPhysicsWorld->setGravity(btVector3(0, -20.0f, 0));
} //InitBulletPhysics

/// Initialize the renderer, the physics engine, and the step timer.

void CGame::Initialize(){
  m_pRenderer = new CRenderer; 
  m_pRenderer->Initialize();
  m_pRenderer->LoadGeometricPrimitives(); //meshes for the spheres and boxes
  
  m_pObjectManager = new CObjectManager; //set up object manager 

  m_pTimer->SetFixedTimeStep();
  m_pTimer->SetFrameTime(1/60.0f);
  
  LoadSounds(); //load the sounds

  InitBulletPhysics(); //init bullet physics engine
  m_pPhysicsWorld->setInternalTickCallback(myTickCallback);

  LBaseCamera* pCamera = m_pRenderer->GetCameraPtr(); //camera pointer
  ResetCamera();
  m_pAudio->SetScale(64.0f);
  m_pAudio->SetListener(pCamera);

  BeginGame();
} //Initialize

/// Initialize the audio player and load game sounds.

void CGame::LoadSounds(){
  m_pAudio->Initialize(eSound::Size);

  m_pAudio->Load(eSound::Clang, "clang");
  m_pAudio->Load(eSound::Click, "click");
  m_pAudio->Load(eSound::TapLight, "taplight");
  m_pAudio->Load(eSound::TapHard, "taphard");
  m_pAudio->Load(eSound::ThumpLight, "thumplight");
  m_pAudio->Load(eSound::ThumpMedium, "thumpmedium");
  m_pAudio->Load(eSound::ThumpHard,"thumphard");
} //LoadSounds

/// Reset Camera to its og position
void CGame::ResetCamera() {
    LBaseCamera* pCamera = m_pRenderer->GetCameraPtr(); //camera pointer
    pCamera->MoveTo(Vector3(0, m_fPlatformHeight + 25.0, -150.0f)); //reset camera position
    pCamera->SetYaw(0.0f);
    pCamera->SetPitch(0.0f);
    pCamera->SetRoll(0.0f);
}


/// Release all of the DirectX12 objects by deleting the renderer.

void CGame::Release(){
  delete m_pRenderer;
  m_pRenderer = nullptr; //for safety
} //Release

/// Create the game objects.

void CGame::CreateObjects(){
  CInstanceDesc d; //mesh instance descriptor

  d.m_eMeshType = eMesh::Box;
  d.m_vExtents = m_vBoxSize;
  d.m_fMass = 0.0f;
  d.m_fRestitution = 0.75f;
  d.m_fFriction = 0.5f;

  for (int i = -1; i < 2; i++) {
      d.m_vPos = Vector3(m_vBoxSize.x*i, m_fPlatformHeight, 0.0f);
      m_pObjectManager->Create(eObject::Box, d);
  }

  return;
} //CreateObjects

/// Start a new game. Clear the object manager and create new game objects.

void CGame::BeginGame(){  
  m_fStartTime = m_pTimer->GetTime(); //mark time
  m_bCollisionSoundsMuted = true; //mute sounds
  m_pObjectManager->Clear(); //clear old objects
  CreateObjects(); //create new objects 
} //BeginGame

/// Keyboard handler.

void CGame::KeyboardHandler(){
  if(m_pKeyboard == nullptr)return; //no keyboard
  if(m_pController && m_pController->IsConnected())return; //use controller

  m_pKeyboard->GetState(); //get current keyboard state 
  
  if(m_pKeyboard->TriggerDown(VK_F1)) //help
    ShellExecute(0, 0, "https://larc.unt.edu/code/physics/block3d/", 
    0, 0, SW_SHOW);

  const float t = m_pTimer->GetFrameTime();
  LBaseCamera* pCamera = m_pRenderer->GetCameraPtr();

  if(m_pKeyboard->TriggerDown(VK_BACK))
    BeginGame();

  if (m_pKeyboard->TriggerDown(VK_SPACE))
      SpawnBlock();
  
  if(pCamera){ //camera navigation
    if(m_pKeyboard->Down(VK_LEFT)) pCamera->AddToYaw(-t/2);
    if(m_pKeyboard->Down(VK_RIGHT))pCamera->AddToYaw(t/2); 
    if(m_pKeyboard->Down(VK_UP))   pCamera->AddToPitch(-t/4);
    if(m_pKeyboard->Down(VK_DOWN)) pCamera->AddToPitch(t/4);
    
    const Vector3 vLookat = pCamera->GetViewVector(); //view vector
    const Vector3 vRight(vLookat.z, 0, -vLookat.x); //right relative to view vector

    //Update impulse values
    bool ImpulseChanged = false;
    if (m_pKeyboard->Down('I')) {
        Impulse += m_fImpulseDelta;
        ImpulseChanged = true;
    }
    if (m_pKeyboard->Down('O')) {
        Impulse -= m_fImpulseDelta;
        ImpulseChanged = true;
    }
    if (Impulse < 0.0) Impulse = 0.0;
    if (Impulse > m_fMaxImpulse) Impulse = m_fMaxImpulse;

    if (m_pKeyboard->TriggerDown('R')) ResetCamera();
    
    //Print new impulse value
    if (ImpulseChanged) {
        char Buff[128];
        sprintf(Buff, "Impulse: %f\n", Impulse);
        OutputDebugString(Buff);
    }

    const float fSpeed = 20.0f; //speed multiplier
    const Vector3 vRightDelta = 0.8f*fSpeed*t*vRight; //position delta rightwards
    if(m_pKeyboard->Down('A'))pCamera->AddToPos(-vRightDelta); //strafe left
    if(m_pKeyboard->Down('D'))pCamera->AddToPos(vRightDelta); //strafe right

    if (m_pKeyboard->Down('Q'))pCamera->AddToPos(Vector3(0.0f, fSpeed * t, 0.0f)); //Up
    if (m_pKeyboard->Down('E'))pCamera->AddToPos(Vector3(0.0f, -fSpeed * t, 0.0f)); //Down
      
    const Vector3 vForwardDelta = fSpeed*t*vLookat; //position delta forwards
    if(m_pKeyboard->Down('W'))pCamera->AddToPos(vForwardDelta); //move forwards
    if(m_pKeyboard->Down('S'))pCamera->AddToPos(-vForwardDelta); //move backwards
  } //if
} //KeyboardHandler

/// Render an animation frame. 

void CGame::RenderFrame(){
  const float t = m_pTimer->GetFrameTime(); //frame time
  m_pRenderer->GetCameraPtr()->Move(t); //move camera

  m_pRenderer->BeginFrame();
  m_pObjectManager->Draw(); //draw objects
  RenderSineIndicator();
  m_pRenderer->EndFrame();
} //RenderFrame

/// Process an animation frame. Handle keyboard and controller input, advance
/// the timer a tick, perform a physics world step, update all objects, and
/// finally render them.

void CGame::ProcessFrame(){
  ProcessGameState();
  KeyboardHandler(); //handle keyboard input
  ProcessSineIndicator();
  

  m_pAudio->BeginFrame(); //notify audio player that frame has begun
  m_pAudio->SetListener(m_pCamera); //move listener to camera

  m_pTimer->Tick([&](){ 
    const float t = m_pTimer->GetFrameTime(); //frame time
    m_pPhysicsWorld->stepSimulation(t, 4); //physics world step
    m_pObjectManager->Update(); //update all objects
  });


  //prevent initial contact sounds from playing

  if(m_pTimer->GetTime() - m_fStartTime > 0.5f)
    m_bCollisionSoundsMuted = false;

  RenderFrame(); //render a frame of animation
} //ProcessFrame

void CGame::SpawnBlock() {
    CInstanceDesc d; //instance descriptor

    d.m_eMeshType = eMesh::Box;
    d.m_vExtents = m_vBoxSize;
    d.m_fMass = 50.0f;
    d.m_fRestitution = 0.0f;
    d.m_fFriction = 0.5f;
    d.m_vPos = SineIndicatorPos-Vector3(0.0f,m_vBoxSize.y,0.0f);
    d.m_fScale = 1000.0f;

    CObject* p = m_pObjectManager->Create(eObject::Box, d); //create it
    p->ApplyImpulse(Vector3(0.0f, -Impulse, 0.0f), Vector3());
    Boxes.push_back(p);
    Score = Boxes.size();
}

void CGame::RenderSineIndicator() {

    CInstanceDesc d;

    d.m_eMeshType = eMesh::Box;
    d.m_vExtents = m_vBoxSize * 0.0f; //no collision
    d.m_fMass = 0.0f;
    d.m_fRestitution = 0.0f;
    d.m_fFriction = 0.0f;
        
        
    float Time = m_pTimer->GetTime();
    float Speedf = 2.0;
    float SineV = sinf(Time * Speedf);
    d.m_vPos = SineIndicatorPos;
        
    //d.m_vPos = CenterPos;
    //SineObj = m_pObjectManager->Create(eObject::Box, d);
    CObject* SineIndicator = new CObject(eObject::Box, d);
    m_pRenderer->Render(SineIndicator);
}

void CGame::ProcessSineIndicator() {
    Vector3 CenterPos = Vector3(0.0f, 75.0f, 0.0f);
    Vector3 Pan = Vector3(50.0f, 0.0f, 0.0f);

    float Time = m_pTimer->GetTime();
    float Speedf = 2.0;
    float SineV = sinf(Time * Speedf);
    SineIndicatorPos = CenterPos + SineV * Pan;
}

void CGame::ProcessGameState() {
    bool KillAll = false;
    for (CObject* Box : Boxes) {
        if (Box->GetPosition().y < m_fPlatformHeight) {
            OutputDebugString("OUT OF BOUNDS");
            KillAll = true;
            break;
        }
    }
    if (KillAll) {
        for (CObject* Box : Boxes) {
            Box->kill();
        }
        //Update high score
        if (Score > HighScore) {
            HighScore = Score;
        }
        Score = 0;
        Boxes.clear();
    }
}

/// 
/// GAMEPLAY:
/// - A dropper moves side-to-side
/// - Hit 'SPACE' to drop
/// - If one falls off, you lose
/// - Q/E to move camera up/down
/// - I/O to increase/decrease impulse from the dropper
/// 
/// 
/// Things to implement:
/// - UI elements to show score, high score, and controls (and a toggle to hide these elements)
/// - Make score based on number of stationary objects, and max it with itself