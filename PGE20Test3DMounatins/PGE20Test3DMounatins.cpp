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
   
    olc::vf3d vf3Up = { 0.0f, 1.0f, 0.0f };           // vf3d up direction
    olc::vf3d vf3Camera = { 0.0f, 0.0f, 0.0f };       // vf3d camera direction
    olc::vf3d vf3LookDir = { 0.0f, 0.0f, 1.0f };    // vf3d look direction
    olc::vf3d vf3Forward = { 0.0f, 0.0f, 0.0f };      // vf3d Forward direction

    float fYaw = 0.0f;		// FPS Camera rotation in XZ plane
    float fTheta = 0.0f;	// Spins World transform


    /* Sprites */
    olc::Sprite* sprOLCPGEMobLogo = nullptr;
    olc::Sprite* sprLandScape = nullptr;
    /* END Sprites*/

    /* Decals */
    olc::Decal* decOLCPGEMobLogo = nullptr;
    olc::Decal* decLandScape = nullptr;
    /* End Decals */


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

        // Called once at the start, so create things here
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
        SetDrawTarget(nullptr);

      
        // New code:
        olc::vf3d  vf3Target = { 0,0,1 };

        olc::mf4d mMovement, mYaw, mOffset, mCollision;

        mMovement.translate(vf3Camera);        // first we move to the new location
        mOffset.translate(0.0, -10.0, 0.0);     // Add our offset
        mMovement.rotateY(fTheta);             // Rotate the camera left/right
        matWorld = mMovement * mOffset;        // Get our new view point
        //TODO: Add mCollision
        //mCollision.translate(0.0f, 0.0f, 0.0f);
        //matWorld = mMovement * mOffset * mCollision;

        mYaw.rotateX(fYaw);                       // Second rotate camera Up/Down
        vf3LookDir = mYaw * vf3Target;            // Get our new direction
        vf3Target = vf3Camera + vf3LookDir;     // Set our target

        matView.pointAt(vf3Camera, vf3Target, vf3Up);   // Point at our Target

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


        HW3D_DrawLine((matView * matWorld).m, { 0.0f, 0.0f, 0.0f }, { 100.0f, 100.0f, 100.0f }, olc::RED);

        HW3D_DrawLineBox((matView * matWorld).m, { 0.0f, 0.0f, 0.0f }, { 10.0f, 10.0f, 10.0f }, olc::YELLOW);


        HW3D_DrawObject((matView * matWorld).m, decLandScape, meshMountain.layout, meshMountain.pos, meshMountain.uv, meshMountain.col);

     
        // End new code

        // Draw Logo
        DrawDecal({ 5.0f, (float)ScreenHeight() - 100 }, decOLCPGEMobLogo, { 0.5f, 0.5f });

        if (GetKey(olc::Key::ESCAPE).bPressed)
        {
            return false;
        }
        else
        {
            return true;
        }
		
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
