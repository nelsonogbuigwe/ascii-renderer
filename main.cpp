#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <map>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Math Library ---

struct Vec3
{
    float x = 0, y = 0, z = 0;

    static Vec3 add(const Vec3 &a, const Vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    static Vec3 subtract(const Vec3 &a, const Vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    static Vec3 scale(const Vec3 &v, float s) { return {v.x * s, v.y * s, v.z * s}; }
    static float dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    static Vec3 cross(const Vec3 &a, const Vec3 &b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }
    static float length(const Vec3 &v) { return std::sqrt(dot(v, v)); }
    static Vec3 normalize(const Vec3 &v)
    {
        float len = length(v);
        return len > 0 ? scale(v, 1.0f / len) : Vec3{0, 0, 0};
    }
};

struct Vec4
{
    float x = 0, y = 0, z = 0, w = 0;
};

struct Mat4
{
    float m[16]; // Column-major order: m[col*4 + row]

    static Mat4 identity()
    {
        Mat4 mat = {};
        mat.m[0] = mat.m[5] = mat.m[10] = mat.m[15] = 1.0f;
        return mat;
    }

    static Mat4 multiply(const Mat4 &a, const Mat4 &b)
    {
        Mat4 result = {};
        for (int i = 0; i < 4; ++i)
        { // Column
            for (int j = 0; j < 4; ++j)
            { // Row
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                {
                    // A's column k, row j multiplied by B's column i, row k
                    sum += a.m[k * 4 + j] * b.m[i * 4 + k];
                }
                result.m[i * 4 + j] = sum;
            }
        }
        return result;
    }

    Vec4 transform(const Vec4 &v) const
    {
        Vec4 result;
        result.x = m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12] * v.w;
        result.y = m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13] * v.w;
        result.z = m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w;
        result.w = m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w;
        return result;
    }

    static Mat4 create_translation(const Vec3 &t)
    {
        Mat4 mat = identity();
        mat.m[12] = t.x;
        mat.m[13] = t.y;
        mat.m[14] = t.z;
        return mat;
    }

    static Mat4 create_rotation_y(float angle_rad)
    {
        Mat4 mat = identity();
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        mat.m[0] = c;
        mat.m[8] = s;
        mat.m[2] = -s;
        mat.m[10] = c;
        return mat;
    }

    static Mat4 perspective(float fov_degrees, float aspect, float near_plane, float far_plane)
    {
        Mat4 result = {};
        float tan_half_fov = std::tan(fov_degrees * M_PI / 360.0f);
        result.m[0] = 1.0f / (aspect * tan_half_fov);
        result.m[5] = 1.0f / tan_half_fov;
        result.m[10] = -(far_plane + near_plane) / (far_plane - near_plane);
        result.m[11] = -1.0f;
        result.m[14] = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
        return result;
    }

    static Mat4 lookAt(const Vec3 &eye, const Vec3 &target, const Vec3 &up)
    {
        Vec3 zaxis = Vec3::normalize(Vec3::subtract(eye, target));
        Vec3 xaxis = Vec3::normalize(Vec3::cross(up, zaxis));
        Vec3 yaxis = Vec3::cross(zaxis, xaxis);

        Mat4 viewMatrix = identity();
        viewMatrix.m[0] = xaxis.x;
        viewMatrix.m[4] = xaxis.y;
        viewMatrix.m[8] = xaxis.z;
        viewMatrix.m[1] = yaxis.x;
        viewMatrix.m[5] = yaxis.y;
        viewMatrix.m[9] = yaxis.z;
        viewMatrix.m[2] = zaxis.x;
        viewMatrix.m[6] = zaxis.y;
        viewMatrix.m[10] = zaxis.z;

        Mat4 translation = create_translation({-eye.x, -eye.y, -eye.z});
        return multiply(viewMatrix, translation);
    }
};

// --- Helper Functions ---

char get_ascii_char(float intensity)
{
    const std::string ascii_chars = " .:-=+*#%@"; // Dark to light
    int index = static_cast<int>(intensity * (ascii_chars.length() - 1));
    index = std::max(0, std::min(static_cast<int>(ascii_chars.length() - 1), index));
    return ascii_chars[index];
}

// Barycentric coordinates calculation
Vec3 barycentric(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c)
{
    Vec3 v0 = Vec3::subtract(b, a);
    Vec3 v1 = Vec3::subtract(c, a);
    Vec3 v2 = Vec3::subtract(p, a);
    float d00 = Vec3::dot(v0, v0);
    float d01 = Vec3::dot(v0, v1);
    float d11 = Vec3::dot(v1, v1);
    float d20 = Vec3::dot(v2, v0);
    float d21 = Vec3::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-5)
        return {-1, -1, -1}; // Degenerate triangle
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;
    return {u, v, w};
}

// --- Main Application ---

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_obj_file> <path_to_font_file>" << std::endl;
        return 1;
    }
    std::string inputfile = argv[1];
    std::string fontfile = argv[2];

    // 1. Initialize SDL and SDL_ttf
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    if (TTF_Init() == -1)
    {
        std::cerr << "SDL_ttf could not initialize! TTF_Error: " << TTF_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    const int SCREEN_WIDTH = 160; // Width in characters
    const int SCREEN_HEIGHT = 90; // Height in characters
    const int FONT_SIZE = 12;     // Font point size
    int font_width, font_height;

    TTF_Font *font = TTF_OpenFont(fontfile.c_str(), FONT_SIZE);
    if (!font)
    {
        std::cerr << "Failed to load font: " << TTF_GetError() << std::endl;
        TTF_Quit();
        SDL_Quit();
        return 1;
    }
    TTF_SizeText(font, " ", &font_width, &font_height); // Get character dimensions

    const int PIXEL_WIDTH = SCREEN_WIDTH * font_width;
    const int PIXEL_HEIGHT = SCREEN_HEIGHT * font_height;

    SDL_Window *window = SDL_CreateWindow("ASCII Renderer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, PIXEL_WIDTH, PIXEL_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!window || !renderer)
    {
        std::cerr << "Window or Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        TTF_CloseFont(font);
        TTF_Quit();
        SDL_Quit();
        return 1;
    }

    // Pre-render ASCII characters to textures for performance
    std::map<char, SDL_Texture *> char_texture_cache;
    SDL_Color text_color = {255, 255, 255, 255}; // White
    const std::string ascii_chars = " .:-=+*#%@";
    for (char c : ascii_chars)
    {
        std::string s(1, c);
        SDL_Surface *text_surface = TTF_RenderText_Solid(font, s.c_str(), text_color);
        if (text_surface)
        {
            char_texture_cache[c] = SDL_CreateTextureFromSurface(renderer, text_surface);
            SDL_FreeSurface(text_surface);
        }
    }

    // 2. Load OBJ Model
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile.c_str()))
    {
        std::cerr << "Failed to load OBJ: " << warn << err << std::endl;
        return 1;
    }

    // 3. Main Loop
    bool quit = false;
    SDL_Event e;
    float rotation_angle_y = 0.0f;

    std::vector<float> depth_buffer(SCREEN_WIDTH * SCREEN_HEIGHT, 0.0f);
    std::vector<char> char_buffer(SCREEN_WIDTH * SCREEN_HEIGHT, ' ');

    Vec3 camera_pos = {0.0f, 2.0f, -5.0f};
    Vec3 look_at = {0.0f, 0.0f, 0.0f};
    Vec3 up_vec = {0.0f, 1.0f, 0.0f};
    Vec3 light_direction = Vec3::normalize({0.5f, -1.0f, -1.0f});

    while (!quit)
    {
        while (SDL_PollEvent(&e) != 0)
        {
            if (e.type == SDL_QUIT)
                quit = true;
        }

        std::fill(depth_buffer.begin(), depth_buffer.end(), 0.0f); // Init with 0 for 1/w
        std::fill(char_buffer.begin(), char_buffer.end(), ' ');

        SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);
        SDL_RenderClear(renderer);

        // 4. Setup Matrices
        Mat4 model_matrix = Mat4::create_rotation_y(rotation_angle_y);
        rotation_angle_y += 0.01f;
        Mat4 view_matrix = Mat4::lookAt(camera_pos, look_at, up_vec);
        Mat4 projection_matrix = Mat4::perspective(90.0f, (float)PIXEL_WIDTH / PIXEL_HEIGHT, 0.1f, 100.0f);

        Mat4 mv_matrix = Mat4::multiply(view_matrix, model_matrix);
        Mat4 mvp_matrix = Mat4::multiply(projection_matrix, mv_matrix);

        // 5. Render Loop
        for (const auto &shape : shapes)
        {
            for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
            {
                int fv = shape.mesh.num_face_vertices[f];
                if (fv != 3)
                    continue; // Only process triangles

                tinyobj::index_t idx[3];
                Vec3 v_world[3];
                for (int i = 0; i < 3; ++i)
                {
                    idx[i] = shape.mesh.indices[f * 3 + i];
                    v_world[i] = {
                        attrib.vertices[3 * idx[i].vertex_index + 0],
                        attrib.vertices[3 * idx[i].vertex_index + 1],
                        attrib.vertices[3 * idx[i].vertex_index + 2]};
                }

                // Back-face culling
                Vec3 edge1 = Vec3::subtract(v_world[1], v_world[0]);
                Vec3 edge2 = Vec3::subtract(v_world[2], v_world[0]);
                Vec3 face_normal = Vec3::normalize(Vec3::cross(edge1, edge2));
                Vec3 view_vector = Vec3::normalize(Vec3::subtract(v_world[0], camera_pos));
                if (Vec3::dot(face_normal, view_vector) >= 0)
                    continue;

                // Flat lighting
                float intensity = Vec3::dot(face_normal, Vec3::scale(light_direction, -1.0f));
                intensity = std::max(0.1f, intensity); // Ambient light
                char ascii_char = get_ascii_char(intensity);

                // Transform vertices
                Vec4 v_clip[3];
                Vec3 v_ndc[3];
                Vec3 v_screen[3];
                float inv_w[3];

                bool behind_camera = false;
                for (int i = 0; i < 3; ++i)
                {
                    v_clip[i] = mvp_matrix.transform({v_world[i].x, v_world[i].y, v_world[i].z, 1.0f});
                    if (v_clip[i].w <= 0)
                    { // Vertex is behind or on the camera plane
                        behind_camera = true;
                        break;
                    }

                    inv_w[i] = 1.0f / v_clip[i].w;
                    v_ndc[i] = {v_clip[i].x * inv_w[i], v_clip[i].y * inv_w[i], v_clip[i].z * inv_w[i]};

                    v_screen[i] = {
                        (v_ndc[i].x + 1.0f) * 0.5f * SCREEN_WIDTH,
                        (1.0f - v_ndc[i].y) * 0.5f * SCREEN_HEIGHT,
                        v_ndc[i].z // Not used in screen space, just for barycentric struct
                    };
                }
                if (behind_camera)
                    continue;

                // Rasterize triangle
                int minX = std::max(0, static_cast<int>(std::min({v_screen[0].x, v_screen[1].x, v_screen[2].x})));
                int maxX = std::min(SCREEN_WIDTH - 1, static_cast<int>(std::ceil(std::max({v_screen[0].x, v_screen[1].x, v_screen[2].x}))));
                int minY = std::max(0, static_cast<int>(std::min({v_screen[0].y, v_screen[1].y, v_screen[2].y})));
                int maxY = std::min(SCREEN_HEIGHT - 1, static_cast<int>(std::ceil(std::max({v_screen[0].y, v_screen[1].y, v_screen[2].y}))));

                for (int y = minY; y <= maxY; ++y)
                {
                    for (int x = minX; x <= maxX; ++x)
                    {
                        Vec3 p = {static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, 0};
                        Vec3 bc = barycentric(p, v_screen[0], v_screen[1], v_screen[2]);

                        if (bc.x < 0 || bc.y < 0 || bc.z < 0)
                            continue;

                        float interpolated_inv_w = bc.x * inv_w[0] + bc.y * inv_w[1] + bc.z * inv_w[2];

                        if (interpolated_inv_w > depth_buffer[y * SCREEN_WIDTH + x])
                        {
                            depth_buffer[y * SCREEN_WIDTH + x] = interpolated_inv_w;
                            char_buffer[y * SCREEN_WIDTH + x] = ascii_char;
                        }
                    }
                }
            }
        }

        // Render the character buffer to the screen
        for (int y = 0; y < SCREEN_HEIGHT; ++y)
        {
            for (int x = 0; x < SCREEN_WIDTH; ++x)
            {
                char c = char_buffer[y * SCREEN_WIDTH + x];
                if (c != ' ' && char_texture_cache.count(c))
                {
                    SDL_Rect dst_rect = {x * font_width, y * font_height, font_width, font_height};
                    SDL_RenderCopy(renderer, char_texture_cache[c], NULL, &dst_rect);
                }
            }
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(10);
    }

    // 6. Cleanup
    for (auto const &[key, val] : char_texture_cache)
    {
        SDL_DestroyTexture(val);
    }
    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();

    return 0;
}