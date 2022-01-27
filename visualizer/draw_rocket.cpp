#include "raylib.h"

#include "raymath.h"        // Required for: MatrixRotateXYZ()
#include <rapidcsv.h>

int main(void)
{
    // Initialization
    rapidcsv::Document csv("sim_data/data.csv");
    auto xCol = csv.GetColumn<double>("x");
    auto yCol = csv.GetColumn<double>("y");
    auto zCol = csv.GetColumn<double>("z");
    auto sCol = csv.GetColumn<double>("s");
    auto r_vect_x = csv.GetColumn<double>("r_vect_x");
    auto r_vect_y = csv.GetColumn<double>("r_vect_y");
    auto r_vect_z = csv.GetColumn<double>("r_vect_z");
    auto r_dot_x = csv.GetColumn<double>("r_dot_x");
    auto r_dot_y = csv.GetColumn<double>("r_dot_y");
    auto r_dot_z = csv.GetColumn<double>("r_dot_z");

    auto timeCol = csv.GetColumn<double>("timestamp");
    int iteration = 0;

    Quaternion rotateQuat = QuaternionFromEuler(-PI/2, 0, 0);

    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    //SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(screenWidth, screenHeight, "Rocket animation");

    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 80.0f, -150.0f };// Camera position perspective
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 30.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera type

    Model model = LoadModel("visualizer/rocket.obj");                  // Load model
    Texture2D texture = LoadTexture("visualizer/carbon-fibre.png");  // Load model texture
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;            // Set map diffuse texture

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        if (iteration >= xCol.size()) {
            iteration = xCol.size() - 1;
        }
        float x = xCol[iteration];
        float y = yCol[iteration];
        float z = zCol[iteration];
        float s = sCol[iteration];

        char pos_buf[256];
        sprintf(pos_buf, "Position: {%f, %f, %f}", r_vect_x[iteration], r_vect_y[iteration], r_vect_z[iteration]);
        char vel_buf[256];
        sprintf(vel_buf, "Velocity: {%f, %f, %f}", r_dot_x[iteration], r_dot_y[iteration], r_dot_z[iteration]);

        double timestamp = timeCol[iteration];
        Quaternion fooQuat{x, z, y, s}; // Change order to change rotation in window
        Matrix rocketMatrix = QuaternionToMatrix(QuaternionMultiply(fooQuat, rotateQuat));
        // Tranformation matrix for rotations
        model.transform = rocketMatrix;

        iteration += 1;
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            // Draw 3D model (recomended to draw 3D always before 2D)
            BeginMode3D(camera);

                DrawModel(model, (Vector3){ 0.0f, -8.0f, 0.0f }, 0.5f, BLACK);   // Draw 3d model with texture
                DrawGrid(10, 10.0f);

            EndMode3D();

            // Draw controls info
            DrawRectangle(30, 370, 260, 70, Fade(GREEN, 0.5f));
            DrawRectangleLines(30, 370, 260, 70, Fade(DARKGREEN, 0.5f));
            DrawText(("Timestamp: " + std::to_string(timestamp)).c_str(), 40, 380, 10, DARKGRAY);
            DrawText(pos_buf, 40, 400, 10, DARKGRAY);
            DrawText(vel_buf, 40, 420, 10, DARKGRAY);


        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    UnloadModel(model);     // Unload model data

    CloseWindow();          // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
