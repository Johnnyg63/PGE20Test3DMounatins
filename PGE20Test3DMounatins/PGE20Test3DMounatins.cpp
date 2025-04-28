
//#define STBI_NO_SIMD // Removes SIMD Support
// SIMD greatly improves the speed of your game
#if defined(__arm__)||(__aarch64__)

// Use Advance SIMD NEON when loading images for STB Default is SSE2 (x86)
#define STBI_NEON

#endif

#define OLC_GFX_OPENGL33
#define OLC_PGE_APPLICATION
#define OLC_IMAGE_STB
#include "olcUTIL_Hardware3D.h"
#include "olcPixelGameEngine.h"
#include <immintrin.h> // For AVX/SSE

// Override base class with your custom functionality
class PGE3DMountains : public olc::PixelGameEngine
{
public:
	PGE3DMountains()
	{
		// Name your application
		sAppName = "PGE 2.0 3D Mountains Example";
	}

    olc::mf4d matWorld;
    olc::mf4d matView;
    olc::mf4d matCube;
    olc::mf4d mSkyCube;
    olc::mf4d mat3SPyrd;
    olc::mf4d matPyrd;
    olc::mf4d matMSphere;
    olc::mf4d matProject;
    olc::utils::hw3d::mesh meshMountain;
   
    olc::vf3d vf3dUp = { 0.0f, 1.0f, 0.0f };         // vf3d up direction
    olc::vf3d vf3dCamera = { -10.0f, 5.5f, 40.0f };     // vf3d camera direction
    olc::vf3d vf3dLookDir = { 0.0f, 0.0f, 1.0f };    // vf3d look direction
    olc::vf3d vf3dForward = { 0.0f, 0.0f, 0.0f };    // vf3d Forward direction
    olc::vf3d vf3dOffset = { -10.0f, 5.5f, 40.0f };       // vf3d Offset
    olc::vf3d vf3dSunLocation = { 100.0f, 100.0f, 100.0f };   // vf3d Sun Location

    olc::vf3d vf3dSanityCubeScale = { 2.0f, 2.0f, 2.0f };        // vf3d SanityCube Scale (in sort its Size)
    olc::vf3d vf3dSanityCubeLocation = { 10.0f, 0.0f, 0.0f };   // vf3d SanityCube Location 
    olc::vf3d vf3dSanityCubeOffset = { 0.0f, 0.0f, 0.0f };      // vf3d SanityCube Offset

    olc::vf3d vf3dSkyCubeScale = { 600.0f, 600.0f, 600.0f };     // vf3d SkyCube Scale (in sort its Size)
    olc::vf3d vf3dSkyCubeLocation = { 0.0f, 0.0f, 0.0f };       // vf3d SkyCube Location 
    olc::vf3d vf3dSkyCubeOffset = { -200.0f, -300.0f, -200.0f }; // vf3d SkyCube Offset

    olc::vf3d vf3dPyramidScale = { 30.0f, 50.0f, 30.0f };   // vf3d Pyramid Scale (in sort its Size)
    olc::vf3d vf3dPyramidLocation = { 0.0f, 0.5f, 11.0f };   // vf3d Pyramid Location 
    olc::vf3d vf3dPyramidOffset = { 0.0f, 10.0f, 0.0f };    // vf3d Pyramid Offset


    olc::vf3d vf3dSphereScale = { 5.0f, 5.0f, 5.0f };   // vf3d Sphere Scale (in sort its Size)
    olc::vf3d vf3dSphereLocation = { 100.0f, 100.0f, 100.0f };   // vf3d Sphere Location 
    olc::vf3d vf3dSphereOffset = { 0.0f, 0.0f, 0.0f };    // vf3d Sphere Offset


    float fYaw = 0.0f;		    // FPS Camera rotation in X plane
    float fYawRoC = 1.0f;	    // fYaw Rate of Change Look Up/Down 
    float fTheta = 0.0f;	    // Spins World transform
    float fThetaRoC = 1.5f;	    // fTheta Rate of Change Spin Left/Right
    float fStrifeRoC = 8.5f;    // Strife Rate of Change, thanks: #Boguslavv
    float fForwardRoC = 8.0f;   // Forward/Backwards Rate of Change

    float fSphereRoC = 0.5f;    // Sphere Rate of Change
    float fSphereRotaotionY = -1.57079633; // Sphere start Y rotation position

    float fJump = vf3dOffset.y;     // Monitors jump height so we can land again
    float fJumpRoC = 4.0f;	// fTheta Rate of Change


    /* Sprites */
    olc::Sprite* sprOLCPGEMobLogo = nullptr;
    olc::Sprite* sprLandScape = nullptr;
    olc::Sprite* sprTestCube = nullptr;
    /* END Sprites*/

    /* Decals */
    olc::Decal* decOLCPGEMobLogo = nullptr;
    olc::Decal* decLandScape = nullptr;
    olc::Decal* decTestCube = nullptr;
    /* End Decals */

    /* Renderables */
    olc::Renderable renTestCube;
    olc::Renderable renBrick;
    olc::Renderable renEarth;
    olc::Renderable renSkyCube;
    /* End Reneders */

    /* SkyCube Stuff*/
    //olc::SkyCubeProperties sSkyCubeProps;
    /* End SkyCube Stuff*/



    /* Vectors */
    std::vector<std::string> vecMessages;
    /* END Vectors*/

    /* vars for displaying messages*/
    uint32_t nFrameCount = 0;
    float fStep = 20;
    olc::vf2d vf2MessPos = { 10.0f, 10.0f };


    /* End Messages */
    olc::vi2d centreScreenPos;

    // 3D Camera
    olc::utils::hw3d::Camera3D Cam3D;


    // Skydomes et Al

    // Sanity Cube
    olc::utils::hw3d::mesh matSanityCube;
    olc::utils::hw3d::mesh matSkyCube;
    olc::utils::hw3d::mesh matTriange;
    olc::utils::hw3d::mesh matPyramid;
    olc::utils::hw3d::mesh mat4SPyramid;
    olc::utils::hw3d::mesh matSphere;


public:
	bool OnUserCreate() override
	{
        float fAspect = float(GetScreenSize().x) / float(GetScreenSize().y); // Width / height 
        float S = 1.0f / (tan(3.14159f * 0.25f));
        float f = 1000.0f;
        float n = 0.1f;

       /* matProject(0, 0) = fAspect; matProject(0, 1) = 0.0f; matProject(0, 2) = 0.0f;	              matProject(0, 3) = 0.0f;
        matProject(1, 0) = 0.0f;    matProject(1, 1) = 1;    matProject(1, 2) = 0.0f;                 matProject(1, 3) = 0.0f;
        matProject(2, 0) = 0.0f;    matProject(2, 1) = 0.0f; matProject(2, 2) = -(f / (f - n));       matProject(2, 3) = -1.0f;
        matProject(3, 0) = 0.0f;    matProject(3, 1) = 0.0f; matProject(3, 2) = -((f * n) / (f - n)); matProject(3, 3) = 0.0f;*/

        matWorld.identity();
        matView.identity();

        Cam3D.SetScreenSize(GetScreenSize()); // SetAspectRatio(fAspect);
        Cam3D.SetClippingPlanes(n, f);
        Cam3D.SetFieldOfView(S);


        auto t = olc::utils::hw3d::LoadObj("assets/objectfiles/mountains.obj");
        if (t.has_value())
        {
            meshMountain = *t;
        }
        else
        {
            int pause = 0; // TODO: Remove. We have an issue
        }

        // Create SanityCube
        matSanityCube = olc::utils::hw3d::CreateSanityCube();
        matTriange = olc::utils::hw3d::CreateTriangle();
        matPyramid = olc::utils::hw3d::Create3SidedPyramid();
        mat4SPyramid = olc::utils::hw3d::Create4SidedPyramid(olc::utils::hw3d::SOLID_TEXTURE);
        matSphere = olc::utils::hw3d::CreateSphere();
        matSkyCube = olc::utils::hw3d::CreateCube(olc::utils::hw3d::LEFT_CROSS_TEXTURE_RECT_MAP);
        //matSkyCube = olc::utils::hw3d::CreateHW3DSkyCube();

        renTestCube.Load("assets/images/sanity_cube.png");

        renSkyCube.Load("assets/images/TestLarge.jpg");
        //renSkyCube.Load("assets/images/DaylightBoxUV.png");
        
        renBrick.Load("assets/images/Brick.png");
        //renBrick.Load("assets/images/GizaTest1.png");
        //renBrick.Load("assets/images/GizaHighRes.jpg");

        renEarth.Load("assets/images/suntexture.jpg");

        Clear(olc::BLUE);

     
        sprOLCPGEMobLogo = new olc::Sprite("assets/images/olcpgemobilelogo.png");
        decOLCPGEMobLogo = new olc::Decal(sprOLCPGEMobLogo);

        sprLandScape = new olc::Sprite("assets/images/MountainTest1.jpg");
        decLandScape = new olc::Decal(sprLandScape);

        centreScreenPos = GetScreenSize();
        centreScreenPos.x = centreScreenPos.x / 2;
        centreScreenPos.y = centreScreenPos.y / 2;

        // SkyCube
        // TODO: Need to move this to a new area....
        /*sSkyCubeProps.sprBack = new olc::Sprite("assets/images/skybox/right.jpg");
        sSkyCubeProps.sprLeft = new olc::Sprite("assets/images/skybox/left.jpg");
        sSkyCubeProps.sprTop = new olc::Sprite("assets/images/skybox/top.jpg");
        sSkyCubeProps.sprBottom = new olc::Sprite("assets/images/skybox/bottom.jpg");
        sSkyCubeProps.sprFront = new olc::Sprite("assets/images/skybox/front.jpg");
        sSkyCubeProps.sprBack = new olc::Sprite("assets/images/skybox/back.jpg");*/



        // Called once at the start, so create things here
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
        SetDrawTarget(nullptr);
             
        // New code:
        olc::mf4d mRotationX, mRotationY, mRotationZ;  // Rotation Matrices
        olc::mf4d mCubeTrans, mCubeScale;
        olc::mf4d mSkyCubeTrans, mSkyCubeScale;
        olc::mf4d mPyramidTrans, mPyramidScale, mPyramidRotationX, mPyramidRotationY, mPyramidRotationZ;
        olc::mf4d mSphereTrans, mSphereScale, mSphereRotationX, mSphereRotationY, mSphereRotationZ;
        olc::mf4d mPosition, mCollision;
        olc::mf4d mMovement, mOffset;


        // Pyramid
        mPyramidTrans.translate(vf3dPyramidLocation);
        mPyramidScale.scale(vf3dPyramidScale);

        mPyramidRotationX.rotateX(0.78539816); // 45' = 0.78539816R
        mPyramidRotationY.rotateY(0.78539816); // 45' = 0.78539816R
        mPyramidRotationZ.rotateZ(0.78539816); // 45' = 0.78539816R

        mat3SPyrd = mPyramidTrans * mPyramidScale * mPyramidRotationX;

        matPyrd = mPyramidTrans * mPyramidScale;

       
        // Sphere
         fSphereRotaotionY += (fSphereRoC * fElapsedTime);
        if (fSphereRotaotionY > 6.28318531) fSphereRotaotionY = 0;
        mSphereTrans.translate(vf3dSphereLocation);
        mSphereScale.scale(vf3dSphereScale);
        mSphereRotationY.rotateY(fSphereRotaotionY);
        mSphereRotationZ.rotateZ(3.14159265);

        matMSphere = mSphereTrans * mSphereScale *mSphereRotationZ; // Rotate the Sphere into the correct North/South pole position
        matMSphere = matMSphere * mSphereRotationY; // Rotate the Sphere so that Ireland and England are facing us


        // Create a "Point At"
        olc::vf3d vf3dTarget = { 0,0,1 };
       
        mRotationY.rotateY(fTheta);  // Left/Right
        mRotationX.rotateX(fYaw);    // Up/Down

        vf3dLookDir = mRotationY * mRotationX * vf3dTarget;   // Left-Right * Up-Down
        vf3dTarget = vf3dCamera + vf3dLookDir;
        

        Cam3D.SetPosition(vf3dCamera);
        Cam3D.SetTarget(vf3dTarget);
        Cam3D.Update();
        matWorld = Cam3D.GetViewMatrix();

        // SkyCube
        mSkyCubeTrans.translate(vf3dSkyCubeOffset + Cam3D.GetPosition());
        mSkyCubeScale.scale(vf3dSkyCubeScale);

        mSkyCube = mSkyCubeTrans * mSkyCubeScale;

        // Manage forward / backwards
        vf3dForward = vf3dLookDir * (fForwardRoC * fElapsedTime);

        HW3D_Projection(Cam3D.GetProjectionMatrix().m);
        //HW3D_Projection(matProject.m);

        // Lighting
       /* olc::vf3d vLight = vf3dSunLocation.norm();
        olc::Pixel pixIllum;
        for (size_t i = 0; i < meshMountain.pos.size(); i += 3)
        {
            const auto& p0 = meshMountain.pos[i + 0];
            const auto& p1 = meshMountain.pos[i + 1];
            const auto& p2 = meshMountain.pos[i + 2];

            olc::vf3d vCross = olc::vf3d(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]).cross(olc::vf3d(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])).norm();

            float illum = std::clamp(vCross.dot(vLight), 0.0f, 1.0f) * 0.6f + 0.4f;
            pixIllum = olc::PixelF(illum, illum, illum, 1.0f);
            meshMountain.col[i + 0] = pixIllum;
            meshMountain.col[i + 1] = pixIllum;
            meshMountain.col[i + 2] = pixIllum;
        }*/


        //HW3D_DrawSkyCube(mSkyCube.m, &sSkyCubeProps, matSkyCube.layout, matSkyCube.pos, matSkyCube.uv, matSkyCube.col);

        // Draw Skycube first
        //HW3D_DrawObject((matWorld * mSkyCube).m, renSkyCube.Decal(), matSkyCube.layout, matSkyCube.pos, matSkyCube.uv, matSkyCube.col);
        
        HW3D_DrawLine((matWorld).m, { 0.0f, 0.0f, 0.0f }, { 100.0f, 100.0f, 100.0f }, olc::RED);

        HW3D_DrawLineBox((matWorld).m, { 0.0f, 0.0f, 0.0f }, { 10.0f, 10.0f, 10.0f }, olc::YELLOW);

        olc::GPUTask_EXT ext;
        ext.cameraPosition = { Cam3D.GetPosition().x, Cam3D.GetPosition().y, Cam3D.GetPosition().z };
        ext.enableLight = true;
        ext.lightColour = olc::WHITE;
        ext.lightmode = 1;
        ext.lightPosition = { vf3dSunLocation.x, vf3dSunLocation.y, vf3dSunLocation.z };

        olc::GPUTask_EXT ext1;
        ext1.cameraPosition = { Cam3D.GetPosition().x, Cam3D.GetPosition().y, Cam3D.GetPosition().z };
        ext1.enableLight = true;
        ext1.lightColour = olc::WHITE;
        ext1.lightmode = 1;
        ext1.lightPosition = { vf3dSunLocation.x, vf3dSunLocation.y, vf3dSunLocation.z };

      
        // HW3D_DrawObject((matWorld * matCube).m, nullptr, matTriange.layout, matTriange.pos, matTriange.uv, matTriange.col);

        // HW3D_DrawObject((matWorld * mat3SPyrd).m, nullptr, matPyramid.layout, matPyramid.pos, matPyramid.uv, matPyramid.col);

        HW3D_DrawObject_extension((matWorld * matPyrd).m, renBrick.Decal(), mat4SPyramid.layout, mat4SPyramid.pos, mat4SPyramid.uv, mat4SPyramid.col, olc::WHITE, ext1, mat4SPyramid.norm);

        HW3D_DrawObject((matWorld * matMSphere).m, renEarth.Decal(), matSphere.layout, matSphere.pos, matSphere.uv, matSphere.col);


        //decLandScape
        HW3D_DrawObject_extension((matWorld).m, decLandScape, meshMountain.layout, meshMountain.pos, meshMountain.uv, meshMountain.col, olc::WHITE, ext, meshMountain.norm);

        // renTestCube.Decal()
        //HW3D_DrawObject((matWorld * matCube).m, renTestCube.Decal(), matSanityCube.layout, matSanityCube.pos, matSanityCube.uv, matSanityCube.col);

        // Draw Logo
        DrawDecal({ 5.0f, (float)ScreenHeight() - 100 }, decOLCPGEMobLogo, { 0.5f, 0.5f });

        UpdateCamByUserInput(fElapsedTime);

        // Display Messages
        DisplayMessages();

        if (GetKey(olc::Key::ESCAPE).bPressed)
        {
            return false;
        }
        else
        {
            return true;
        }

       
       
		
	}

    /*
    * Updates the cam position by user input
    * Mouse/Touch/Keyboard
    */
    void UpdateCamByUserInput(float fElapsedTime)
    {
        // Handle Camera
        // Touch zeros (single touch) handles Camera look direction
        if (GetMouse(0).bHeld)
        {

            // We know the Right Center point we need to compare our positions
            // Looking Right
            if ((float)GetMousePos().x > (((float)centreScreenPos.x / 100) * 130))
            {
                fTheta -= fThetaRoC * fElapsedTime;


            }

            // Looking Left
            if ((float)GetMousePos().x < (((float)centreScreenPos.x / 100) * 70))
            {
                fTheta += fThetaRoC * fElapsedTime;


            }

            // Looking Up
            if ((float)GetMousePos().y < (((float)centreScreenPos.y / 100) * 70))
            {
                fYaw -= fYawRoC * fElapsedTime;
                if (fYaw < -1.0f) fYaw = -1.0f;
            }

            // Looking Down
            if ((float)GetMousePos().y > (((float)centreScreenPos.y / 100) * 130))
            {
                fYaw += fYawRoC * fElapsedTime;
                if (fYaw > 1.0f) fYaw = 1.0f;
            }

        }
        else
        {
            // Move the camera back to center, stops the dizzies!
            if (fYaw > -0.01f && fYaw < 0.01f)
            {
                fYaw = 0.0f;
            }
            if (fYaw >= 0.01)
            {
                fYaw -= fYawRoC * fElapsedTime;

            }
            if (fYaw <= -0.01)
            {
                fYaw += fYawRoC * fElapsedTime;

            }

        }

        // Handle movement
        // Moving Forward
        if (GetKey(olc::Key::UP).bHeld || GetMouse(1).bHeld)
        {
            vf3dCamera += vf3dForward;
        }

        // Moving Backward
        if (GetKey(olc::Key::DOWN).bHeld)
        {
            vf3dCamera -= vf3dForward;
        }

        // Moving Left (Strife)
        if (GetKey(olc::Key::LEFT).bHeld)
        {
            vf3dCamera.x -= cos(fTheta) * fStrifeRoC * fElapsedTime;
            vf3dCamera.z -= sin(fTheta) * fStrifeRoC * fElapsedTime;
        }


        // Moving Right (Strife)
        if (GetKey(olc::Key::RIGHT).bHeld)
        {
            vf3dCamera.x += cos(fTheta) * fStrifeRoC * fElapsedTime;
            vf3dCamera.z += sin(fTheta) * fStrifeRoC * fElapsedTime;

        }


        // Moving UP
        if (GetKey(olc::Key::SPACE).bHeld)
        {
            fJump += fJumpRoC * fElapsedTime;
            vf3dCamera.y = fJump;
        }
        else if (GetKey(olc::Key::B).bHeld)
        {
            fJump -= fJumpRoC * fElapsedTime;
            vf3dCamera.y = fJump;

        }
        else
        {
           /* if (fJump > (vf3dOffset.y - 0.01f) && fJump < (vf3dOffset.y + 0.01f))
            {
                fJump = vf3dOffset.y;
                vf3dCamera.y = fJump;
            }
            if (fJump >= (vf3dOffset.y + 0.01))
            {
                fJump -= 4.0f * fElapsedTime;
                vf3dCamera.y = fJump;
            }
            if (fJump <= (vf3dOffset.y - 0.01))
            {
                fJump += 4.0f * fElapsedTime;
                vf3dCamera.y = fJump;
            }*/
        }



        // Set Sun Location
        if (GetKey(olc::Key::S).bHeld)
        {
            vf3dSunLocation.x = float(GetMouseX());
            vf3dSunLocation.y = float(GetMouseY());
           
        }

       /* vf3dSphereLocation.x = vf3dSunLocation.x;
        vf3dSphereLocation.y = vf3dSunLocation.y;
        vf3dSphereLocation.z = vf3dSunLocation.z;*/
    }

    /*
    * Displays messages on the screen
    */
    void DisplayMessages()
    {
        nFrameCount = GetFPS();

        std::string sMessage = "OneLoneCoder.com";
        vecMessages.push_back(sMessage);

        sMessage = sAppName + " - FPS: " + std::to_string(nFrameCount);
        vecMessages.push_back(sMessage);

        sMessage = "Sun X: " + std::to_string(vf3dSunLocation.x);
        vecMessages.push_back(sMessage);
        sMessage = "Sun Y: " + std::to_string(vf3dSunLocation.y);
        vecMessages.push_back(sMessage);
        sMessage = "Sun Z: " + std::to_string(vf3dSunLocation.z);
        vecMessages.push_back(sMessage);


        sMessage = "---";
        vecMessages.push_back(sMessage);

        fStep = 10;
        vf2MessPos.y = fStep;
        for (auto& s : vecMessages)
        {
            DrawStringDecal(vf2MessPos, s);
            vf2MessPos.y += fStep;
        }
        vecMessages.clear();


    }
};

int main()
{
	PGE3DMountains demo;

	// Lets do HD!
	if (demo.Construct(1280, 720, 1, 1, false))
		demo.Start();
	return 0;
}
