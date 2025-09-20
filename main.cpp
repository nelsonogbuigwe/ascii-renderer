
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <string>

struct Vec3
{
    float x, y, z;
};

struct Face
{
    int v1, v2, v3;
};

// Load OBJ file (vertices and faces only)
bool loadOBJ(const std::string &filename, std::vector<Vec3> &vertices, std::vector<Face> &faces)
{
    std::ifstream file(filename);
    if (!file.is_open())
        return false;
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "v")
        {
            Vec3 v;
            iss >> v.x >> v.y >> v.z;
            vertices.push_back(v);
        }
        else if (type == "f")
        {
            Face f;
            char slash;
            int ignore;
            // Only support faces with 3 vertices (triangles)
            if (line.find('/') != std::string::npos)
            {
                // f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
                sscanf(line.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d", &f.v1, &ignore, &ignore, &f.v2, &ignore, &ignore, &f.v3, &ignore, &ignore);
            }
            else
            {
                // f v1 v2 v3
                iss >> f.v1 >> f.v2 >> f.v3;
            }
            faces.push_back(f);
        }
    }
    return true;
}

// Rotate a point around Y axis
Vec3 rotateY(const Vec3 &v, float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
}

// Project 3D point to 2D screen
void project(const Vec3 &v, int width, int height, float scale, int &sx, int &sy)
{
    sx = static_cast<int>((v.x * scale) + width / 2);
    sy = static_cast<int>((-v.y * scale) + height / 2);
}

// Draw a point in the buffer
void drawPoint(std::vector<std::string> &buffer, int x, int y, char c)
{
    if (y >= 0 && y < (int)buffer.size() && x >= 0 && x < (int)buffer[0].size())
    {
        buffer[y][x] = c;
    }
}

// Bresenham's line algorithm
void drawLine(std::vector<std::string> &buffer, int x0, int y0, int x1, int y1, char c)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (true)
    {
        drawPoint(buffer, x0, y0, c);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void clearScreen()
{
    // ANSI escape code to clear screen and move cursor to top left
    std::cout << "\033[2J\033[H";
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file.obj>\n";
        return 1;
    }
    std::string filename = argv[1];
    std::vector<Vec3> vertices;
    std::vector<Face> faces;
    if (!loadOBJ(filename, vertices, faces))
    {
        std::cerr << "Failed to load OBJ file: " << filename << "\n";
        return 1;
    }

    const int width = 80;
    const int height = 40;
    const float scale = 15.0f;
    float angle = 0.0f;

    while (true)
    {
        std::vector<std::string> buffer(height, std::string(width, ' '));
        // Rotate and project all vertices
        std::vector<Vec3> transformed;
        for (const auto &v : vertices)
        {
            transformed.push_back(rotateY(v, angle));
        }
        // Draw faces as lines
        for (const auto &f : faces)
        {
            int idx1 = f.v1 - 1, idx2 = f.v2 - 1, idx3 = f.v3 - 1;
            int x1, y1, x2, y2, x3, y3;
            project(transformed[idx1], width, height, scale, x1, y1);
            project(transformed[idx2], width, height, scale, x2, y2);
            project(transformed[idx3], width, height, scale, x3, y3);
            drawLine(buffer, x1, y1, x2, y2, '#');
            drawLine(buffer, x2, y2, x3, y3, '#');
            drawLine(buffer, x3, y3, x1, y1, '#');
        }
        clearScreen();
        for (const auto &line : buffer)
            std::cout << line << '\n';
        angle += 0.05f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
