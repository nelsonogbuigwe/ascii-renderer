#include <SFML/Graphics.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

struct Point3D {
    float x, y, z;
};

struct Model {
    std::vector<Point3D> vertices;

    void rotateY(float angle) {
        float cosTheta = cos(angle);
        float sinTheta = sin(angle);
        for (auto &vertex : vertices) {
            float x = vertex.x * cosTheta - vertex.z * sinTheta;
            float z = vertex.x * sinTheta + vertex.z * cosTheta;
            vertex.x = x;
            vertex.z = z;
        }
    }
};

Model loadModelFromFile(const std::string &filename) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str())) {
        throw std::runtime_error(warn + err);
    }

    Model model;
    for (const auto &shape : shapes) {
        for (const auto &index : shape.mesh.indices) {
            Point3D vertex = {
                    attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]
            };
            model.vertices.push_back(vertex);
        }
    }
    return model;
}

char mapDepthToChar(float depth) {
    const std::string chars = " .:-=+*#%@";
    int index = static_cast<int>((depth + 1) * 0.5 * (chars.size() - 1));
    return chars[std::min(std::max(index, 0), static_cast<int>(chars.size() - 1))];
}

void calculateBoundingBox(const Model &model, Point3D &min, Point3D &max) {
    if (model.vertices.empty()) return;

    min = max = model.vertices[0];
    for (const auto &vertex : model.vertices) {
        if (vertex.x < min.x) min.x = vertex.x;
        if (vertex.y < min.y) min.y = vertex.y;
        if (vertex.z < min.z) min.z = vertex.z;
        if (vertex.x > max.x) max.x = vertex.x;
        if (vertex.y > max.y) max.y = vertex.y;
        if (vertex.z > max.z) max.z = vertex.z;
    }
}

void drawLine(int x0, int y0, int x1, int y1, float z0, float z1, std::vector<std::vector<char>> &frameBuffer, std::vector<std::vector<float>> &zBuffer, char charToDraw) {
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
        std::swap(z0, z1);
    }
    int dx = x1 - x0;
    int dy = std::abs(y1 - y0);
    float dz = (z1 - z0) / dx;
    int error = dx / 2;
    int ystep = (y0 < y1) ? 1 : -1;
    int y = y0;
    float z = z0;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            if (x >= 0 && x < zBuffer.size() && y >= 0 && y < zBuffer[0].size() && z > zBuffer[y][x]) {
                frameBuffer[y][x] = charToDraw;
                zBuffer[y][x] = z;
            }
        } else {
            if (y >= 0 && y < zBuffer.size() && x >= 0 && x < zBuffer[0].size() && z > zBuffer[y][x]) {
                frameBuffer[y][x] = charToDraw;
                zBuffer[y][x] = z;
            }
        }
        z += dz;
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

void rasterizeTriangle(Point3D v0, Point3D v1, Point3D v2, std::vector<std::vector<char>> &frameBuffer, std::vector<std::vector<float>> &zBuffer, char charToDraw) {
    if (v0.y > v1.y) std::swap(v0, v1);
    if (v0.y > v2.y) std::swap(v0, v2);
    if (v1.y > v2.y) std::swap(v1, v2);

    int totalHeight = v2.y - v0.y;
    for (int i = 0; i < totalHeight; i++) {
        bool secondHalf = i > v1.y - v0.y || v1.y == v0.y;
        int segmentHeight = secondHalf ? v2.y - v1.y : v1.y - v0.y;
        float alpha = (float)i / totalHeight;
        float beta = (float)(i - (secondHalf ? v1.y - v0.y : 0)) / segmentHeight;
        Point3D A = { v0.x + (v2.x - v0.x) * alpha, v0.y + (v2.y - v0.y) * alpha, v0.z + (v2.z - v0.z) * alpha };
        Point3D B = secondHalf ? Point3D{ v1.x + (v2.x - v1.x) * beta, v1.y + (v2.y - v1.y) * beta, v1.z + (v2.z - v1.z) * beta }
                               : Point3D{ v0.x + (v1.x - v0.x) * beta, v0.y + (v1.y - v0.y) * beta, v0.z + (v1.z - v0.z) * beta };
        if (A.x > B.x) std::swap(A, B);
        for (int j = A.x; j <= B.x; j++) {
            float phi = B.x == A.x ? 1.0 : (float)(j - A.x) / (float)(B.x - A.x);
            Point3D P = { A.x + (B.x - A.x) * phi, A.y + (B.y - A.y) * phi, A.z + (B.z - A.z) * phi };
            if (P.x >= 0 && P.x < frameBuffer[0].size() && P.y >= 0 && P.y < frameBuffer.size() && P.z > zBuffer[P.y][P.x]) {
                frameBuffer[P.y][P.x] = charToDraw;
                zBuffer[P.y][P.x] = P.z;
            }
        }
    }
}

void renderModel(const Model &model, int width, int height, sf::RenderWindow &window) {
    std::vector<std::vector<char>> frameBuffer(height, std::vector<char>(width, ' '));
    std::vector<std::vector<float>> zBuffer(height, std::vector<float>(width, -std::numeric_limits<float>::infinity()));

    Point3D min, max;
    calculateBoundingBox(model, min, max);

    float scaleX = (width - 1.0f) / (max.x - min.x);
    float scaleY = (height - 1.0f) / (max.y - min.y);
    float scale = std::min(scaleX, scaleY);
    float offsetX = (width - (max.x - min.x) * scale) / 2 - min.x * scale;
    float offsetY = (height - (max.y - min.y) * scale) / 2 - min.y * scale;

    std::vector<Point3D> transformedVertices = model.vertices;

    for (size_t i = 0; i < transformedVertices.size(); i += 3) {
        Point3D v0 = transformedVertices[i];
        Point3D v1 = transformedVertices[i + 1];
        Point3D v2 = transformedVertices[i + 2];

        v0.x = v0.x * scale + offsetX;
        v0.y = v0.y * scale + offsetY;
        v1.x = v1.x * scale + offsetX;
        v1.y = v1.y * scale + offsetY;
        v2.x = v2.x * scale + offsetX;
        v2.y = v2.y * scale + offsetY;

        char charToDraw = mapDepthToChar((v0.z + v1.z + v2.z) / 3.0f);

        rasterizeTriangle(v0, v1, v2, frameBuffer, zBuffer, charToDraw);
    }

    window.clear();
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font" << std::endl;
        return;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            sf::Text text(std::string(1, frameBuffer[y][x]), font);
            text.setCharacterSize(12);
            text.setFillColor(sf::Color::White);
            text.setPosition(x * 12, y * 12);
            window.draw(text);
        }
    }
    window.display();
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <model.obj>" << std::endl;
        return 1;
    } else if (argc > 2) {
        std::cerr << "Too many arguments" << std::endl;
        return 1;
    }

    std::string filename = argv[1];

    Model model;
    try {
        model = loadModelFromFile(filename);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load model: " << e.what() << std::endl;
        return 1;
    }

    sf::RenderWindow window(sf::VideoMode(1000, 850), "ASCII Renderer");
    window.setFramerateLimit(60);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::Resized) {
                sf::View view = window.getView();
                view.setSize(event.size.width, event.size.height);
                window.setView(view);
            }
        }

        model.rotateY(0.01f);

        renderModel(model, window.getSize().x / 12, window.getSize().y / 12, window);

        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}