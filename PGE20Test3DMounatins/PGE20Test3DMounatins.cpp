
#define OLC_GFX_OPENGL33
#define OLC_PGE_APPLICATION
#include "olcUTIL_Hardware3D.h"
#include "olcPixelGameEngine.h"

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
    olc::mf4d matProject;
    olc::utils::hw3d::mesh meshMountain;
   
    olc::vf3d vf3dUp = { 0.0f, 1.0f, 0.0f };         // vf3d up direction
    olc::vf3d vf3dCamera = { 0.0f, 30.0f, 0.0f };     // vf3d camera direction
    olc::vf3d vf3dLookDir = { 0.0f, 0.0f, 1.0f };    // vf3d look direction
    olc::vf3d vf3dForward = { 0.0f, 0.0f, 0.0f };    // vf3d Forward direction
    olc::vf3d vf3dOffset = { 0.0f, 10.0f, 0.0f };   // vf3d Offset

    float fYaw = 0.0f;		// FPS Camera rotation in X plane
    float fYawRoC = 1.0f;	// fYaw Rate of Change
    float fTheta = 0.0f;	// Spins World transform
    float fThetaRoC = 1.5f;	// fTheta Rate of Change
    float fStrifeRoC = 25.0f; // Strife Rate of Change, thanks: #Boguslavv


    float fJump = vf3dOffset.y;     // Monitors jump height so we can land again
    float fJumpRoC = 4.0f;	// fTheta Rate of Change


    /* Sprites */
    olc::Sprite* sprOLCPGEMobLogo = nullptr;
    olc::Sprite* sprLandScape = nullptr;
    /* END Sprites*/

    /* Decals */
    olc::Decal* decOLCPGEMobLogo = nullptr;
    olc::Decal* decLandScape = nullptr;
    /* End Decals */

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

public:
	bool OnUserCreate() override
	{
        float fAspect = float(GetScreenSize().y) / float(GetScreenSize().x);
        float S = 1.0f / (tan(3.14159f * 0.25f));
        float f = 1000.0f;
        float n = 0.1f;

        matProject(0, 0) = fAspect; matProject(0, 1) = 0.0f; matProject(0, 2) = 0.0f;	              matProject(0, 3) = 0.0f;
        matProject(1, 0) = 0.0f;    matProject(1, 1) = 1;    matProject(1, 2) = 0.0f;                 matProject(1, 3) = 0.0f;
        matProject(2, 0) = 0.0f;    matProject(2, 1) = 0.0f; matProject(2, 2) = -(f / (f - n));       matProject(2, 3) = -1.0f;
        matProject(3, 0) = 0.0f;    matProject(3, 1) = 0.0f; matProject(3, 2) = -((f * n) / (f - n)); matProject(3, 3) = 0.0f;

        matWorld.identity();
        matView.identity();

        auto t = olc::utils::hw3d::LoadObj("assets/objectfiles/mountains.obj");
        if (t.has_value())
        {
            meshMountain = *t;
        }
        else
        {
            int pause = 0; // TODO: Remove. We have an issue
        }

        Clear(olc::BLUE);

     
        sprOLCPGEMobLogo = new olc::Sprite("assets/images/olcpgemobilelogo.png");
        decOLCPGEMobLogo = new olc::Decal(sprOLCPGEMobLogo);

        sprLandScape = new olc::Sprite("assets/images/MountainTest1.jpg");
        decLandScape = new olc::Decal(sprLandScape);

        centreScreenPos = GetScreenSize();
        centreScreenPos.x = centreScreenPos.x / 2;
        centreScreenPos.y = centreScreenPos.y / 2;

        // Called once at the start, so create things here
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
        SetDrawTarget(nullptr);

             
        // New code:
        olc::mf4d mRotationX, mRotationY, mRotationZ;  // Rotation Matrices
        olc::mf4d mTrans, matCameraRot, matCamera;
        olc::mf4d mPosition, mCollision;

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

        // Manage forward / backwards
        vf3dForward = vf3dLookDir * (8.0f * fElapsedTime);

        ClearBuffer(olc::CYAN, true);


        HW3D_Projection(matProject.m);

        // Lighting
        for (size_t i = 0; i < meshMountain.pos.size(); i += 3)
        {
            const auto& p0 = meshMountain.pos[i + 0];
            const auto& p1 = meshMountain.pos[i + 1];
            const auto& p2 = meshMountain.pos[i + 2];

            olc::vf3d vCross = olc::vf3d(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]).cross(olc::vf3d(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])).norm();

            olc::vf3d vLight = olc::vf3d(1.0f, 1.0f, 1.0f).norm();

            float illum = std::clamp(vCross.dot(vLight), 0.0f, 1.0f) * 0.6f + 0.4f;
            meshMountain.col[i + 0] = olc::PixelF(illum, illum, illum, 1.0f);
            meshMountain.col[i + 1] = olc::PixelF(illum, illum, illum, 1.0f);
            meshMountain.col[i + 2] = olc::PixelF(illum, illum, illum, 1.0f);
        }

        HW3D_DrawLine((matWorld).m, { 0.0f, 0.0f, 0.0f }, { 100.0f, 100.0f, 100.0f }, olc::RED);

        HW3D_DrawLineBox((matWorld).m, { 0.0f, 0.0f, 0.0f }, { 10.0f, 10.0f, 10.0f }, olc::YELLOW);

        HW3D_DrawObject((matWorld).m, decLandScape, meshMountain.layout, meshMountain.pos, meshMountain.uv, meshMountain.col);

     
        // End new code

        // Draw Logo
        DrawDecal({ 5.0f, (float)ScreenHeight() - 100 }, decOLCPGEMobLogo, { 0.5f, 0.5f });

        // Handle Camera
         // Touch zeros (single touch) handles Camera look direction
        if (GetMouse(0).bHeld)
        {

            // We know the Right Center point we need to compare our positions
            // Looking Right
            if ((float)GetMousePos().x > (((float)centreScreenPos.x / 100) * 130))
            {
                fTheta += fThetaRoC * fElapsedTime;


            }

            // Looking Left
            if ((float)GetMousePos().x < (((float)centreScreenPos.x / 100) * 70))
            {
                fTheta -= fThetaRoC * fElapsedTime;


            }

            // Looking Up
            if ((float)GetMousePos().y < (((float)centreScreenPos.y / 100) * 70))
            {
                fYaw += fYawRoC * fElapsedTime;
                if (fYaw > 1.0f) fYaw = 1.0f;
                

            }

            // Looking Down
            if ((float)GetMousePos().y > (((float)centreScreenPos.y / 100) * 130))
            {
                fYaw -= fYawRoC * fElapsedTime;
                if (fYaw < -1.0f) fYaw = -1.0f;

            }

        }
        else
        {
            // Move the camera back to centre, stops the dizzies!
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
            //vf3dCamera.z += 8.0f * fElapsedTime;
            vf3dCamera -= vf3dForward;
        }

        // Moving Backward
        if (GetKey(olc::Key::DOWN).bHeld)
        {
            //vf3dCamera.z -= 8.0f * fElapsedTime;
            vf3dCamera += vf3dForward;
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
        else
        {
            if (fJump > (vf3dOffset.y - 0.01f) && fJump < (vf3dOffset.y + 0.01f))
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
            }
        }


        // Moving Down
        if (GetKey(olc::Key::B).bHeld)
        {
            fJump += 4.0f * fElapsedTime;
            vf3dCamera.y = fJump;
            // TODO: add condition code to stop down movement when jump = 0
        }

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
    * Displays messages on the screen
    */
    void DisplayMessages()
    {
        nFrameCount = GetFPS();

        std::string sMessage = "OneLoneCoder.com";
        vecMessages.push_back(sMessage);

        sMessage = sAppName + " - FPS: " + std::to_string(nFrameCount);
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
