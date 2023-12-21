#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#define PI 3.14159265 

sf::Vector2f perpendicular_vector(const sf::Vector2f& v) {
    return sf::Vector2f(-v.y, v.x);
}

enum class MovementPolygon {
    NONE = 0,
    FORWARD,
    BACKWARD,
};
enum class DirectionPolygon {
    NONE = 0,
    LEFT,
    RIGHT
};

sf::Vector2f calculateDirection(float rotationAngle)
{
    float radians = static_cast<float>(rotationAngle * (PI / 180.0f));
    return -perpendicular_vector(sf::Vector2f(cos(radians), sin(radians)));
}


class RegularPolygon : public sf::ConvexShape 
{
private:
    unsigned int sideCount;
    float sideLength;
    sf::Vector2f centroid;

    void updateShape() {
        setPointCount(sideCount);

        float angleOffset = static_cast<float>((((sideCount - 2) * 180) / sideCount) / 2);
        float shapeAngle = 360.0f / sideCount;

        for (unsigned int i = 0; i < sideCount; ++i) {
            float angle = angleOffset + shapeAngle * i;
            float radians = static_cast<float>(angle * (PI / 180.0));
            sf::Vector2f vertexPosition(std::cos(radians) * sideLength, std::sin(radians) * sideLength);
            setPoint(i, vertexPosition);
            centroid += getPoint(i);
        }
        centroid /= static_cast<float>(sideCount);
        setOrigin(centroid);
    }

public:
    bool overlap = false;
    RegularPolygon(unsigned int sideCount, float sideLength) :
        sideCount(sideCount),
        sideLength(sideLength),
        centroid(0, 0)
    {
        setOutlineThickness(1);
        setFillColor(sf::Color::Transparent);
        updateShape();
    }  

    sf::Vector2f getCentroid()
    {
        return centroid;
    }
};

bool isSAT(RegularPolygon& r1, RegularPolygon& r2)
{
    auto* poly1 = &r1;
    auto* poly2 = &r2;

    for (int shape = 0; shape < 2; shape++)
    {
        if (shape == 1)
        {
            poly1 = &r2;
            poly2 = &r1;
        }
        for (int a = 0; a < poly1->getPointCount(); a++)
        {
            int b = (a + 1) % poly1->getPointCount();
            sf::Vector2f axisProj = perpendicular_vector(poly1->getPoint(b) - poly1->getPoint(a));
            //{ -(poly1->at(b).y - poly1->at(a).y), (poly1->at(b).x - poly1->at(a).x) };

            //work out min and max for r1
            float min_r1 = INFINITY;
            float max_r1 = -INFINITY;
            for (int p = 0; p < poly1->getPointCount(); p++)
            {
                float q = (poly1->getTransform().transformPoint(poly1->getPoint(p)).x * axisProj.x + poly1->getTransform().transformPoint(poly1->getPoint(p)).y * axisProj.y);
                min_r1 = std::min(min_r1, q);
                max_r1 = std::max(max_r1, q);
            }

            //work out min and max for r2
            float min_r2 = INFINITY;
            float max_r2 = -INFINITY;
            for (int p = 0; p < poly2->getPointCount(); p++)
            {
                float q = (poly2->getTransform().transformPoint(poly2->getPoint(p)).x * axisProj.x + poly2->getTransform().transformPoint(poly2->getPoint(p)).y * axisProj.y);
                min_r2 = std::min(min_r2, q);
                max_r2 = std::max(max_r2, q);
            }

            if (!(max_r2 >= min_r1 && max_r1 >= min_r2))
            {
                return false;
            }
        }
    }
    return true;
}

bool isSAT_STATIC(RegularPolygon& r1, RegularPolygon& r2)
{
    auto* poly1 = &r1;
    auto* poly2 = &r2;

    float overlap = INFINITY;
    for (int shape = 0; shape < 2; shape++)
    {
        if (shape == 1)
        {
            poly1 = &r2;
            poly2 = &r1;
        }
        for (int a = 0; a < poly1->getPointCount(); a++)
        {
            int b = (a + 1) % poly1->getPointCount();
            sf::Vector2f axisProj = perpendicular_vector(poly1->getPoint(b) - poly1->getPoint(a));
            //{ -(poly1->at(b).y - poly1->at(a).y), (poly1->at(b).x - poly1->at(a).x) };


            //work out min and max for r1
            float min_r1 = INFINITY;
            float max_r1 = -INFINITY;
            for (int p = 0; p < poly1->getPointCount(); p++)
            {
                float q = (poly1->getTransform().transformPoint(poly1->getPoint(p)).x * axisProj.x + poly1->getTransform().transformPoint(poly1->getPoint(p)).y * axisProj.y);
                min_r1 = std::min(min_r1, q);
                max_r1 = std::max(max_r1, q);
            }

            //work out min and max for r2
            float min_r2 = INFINITY;
            float max_r2 = -INFINITY;
            for (int p = 0; p < poly2->getPointCount(); p++)
            {
                float q = (poly2->getTransform().transformPoint(poly2->getPoint(p)).x * axisProj.x + poly2->getTransform().transformPoint(poly2->getPoint(p)).y * axisProj.y);
                min_r2 = std::min(min_r2, q);
                max_r2 = std::max(max_r2, q);
            }

            overlap = std::min(std::min(max_r1, max_r2) - std::max(min_r1, min_r2), overlap);
            if (!(max_r2 >= min_r1 && max_r1 >= min_r2))
            {
                return false;
            }
        }
    }

    sf::Vector2f d = { r2.getPosition().x - r1.getPosition().x,r2.getPosition().y - r1.getPosition().y };
    float s = sqrtf(d.x * d.x + d.y * d.y);

    auto pos = r1.getPosition();
    pos.x -= overlap * d.x / s;
    pos.y -= overlap * d.y / s;

    //r1.setPosition(r1.getPosition().x - (overlap * d.x / s), r1.getPosition().y - (overlap * d.y / s));
    r1.setPosition(pos);

    std::cout << r1.getPosition().x << ' ' << r1.getPosition().y << std::endl;
    return false;
}

bool isDIAG(RegularPolygon& r1, RegularPolygon& r2)
{
    auto* poly1 = &r1;
    auto* poly2 = &r2;

    float overlap = INFINITY;
    for (int shape = 0; shape < 2; shape++)
    {
        if (shape == 1)
        {
            poly1 = &r2;
            poly2 = &r1;
        }
        for (int p = 0; p < poly1->getPointCount(); p++)
        {

            sf::Vector2f line_r1s = poly1->getPosition();
            sf::Vector2f line_r1e = poly1->getTransform().transformPoint(poly1->getPoint(p));

            for (int q = 0; q < poly2->getPointCount(); q++)
            {
                sf::Vector2f line_r2s = poly2->getTransform().transformPoint(poly2->getPoint(q));
                sf::Vector2f line_r2e = poly2->getTransform().transformPoint(poly2->getPoint((q + 1) % poly2->getPointCount()));


                float h = (line_r2e.x - line_r2s.x) * (line_r1s.y - line_r1e.y) - (line_r1s.x - line_r1e.x) * (line_r2e.y - line_r2s.y);

                float t1 = ((line_r2s.y - line_r2e.y) * (line_r1s.x - line_r2s.x) + (line_r2e.x - line_r2s.x) * (line_r1s.y - line_r2s.y)) / h;
                float t2 = ((line_r1s.y - line_r1e.y) * (line_r1s.x - line_r2s.x) + (line_r1e.x - line_r1s.x) * (line_r1s.y - line_r2s.y)) / h;

                if (t1 >= 0.0f && t1 < 1.0f && t2 >= 0.0f && t2 < 1.0f)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool isDIAG_STATIC(RegularPolygon& r1, RegularPolygon& r2)
{
    auto* poly1 = &r1;
    auto* poly2 = &r2;

    float overlap = INFINITY;
    for (int shape = 0; shape < 2; shape++)
    {
        if (shape == 1)
        {
            poly1 = &r2;
            poly2 = &r1;
        }
        for (int p = 0; p < poly1->getPointCount(); p++)
        {

            sf::Vector2f line_r1s = poly1->getPosition();
            sf::Vector2f line_r1e = poly1->getTransform().transformPoint(poly1->getPoint(p));

            sf::Vector2f disp(0, 0);

            for (int q = 0; q < poly2->getPointCount(); q++)
            {
                sf::Vector2f line_r2s = poly2->getTransform().transformPoint(poly2->getPoint(q));
                sf::Vector2f line_r2e = poly2->getTransform().transformPoint(poly2->getPoint((q + 1) % poly2->getPointCount()));


                float h = (line_r2e.x - line_r2s.x) * (line_r1s.y - line_r1e.y) - (line_r1s.x - line_r1e.x) * (line_r2e.y - line_r2s.y);

                float t1 = ((line_r2s.y - line_r2e.y) * (line_r1s.x - line_r2s.x) + (line_r2e.x - line_r2s.x) * (line_r1s.y - line_r2s.y)) / h;
                float t2 = ((line_r1s.y - line_r1e.y) * (line_r1s.x - line_r2s.x) + (line_r1e.x - line_r1s.x) * (line_r1s.y - line_r2s.y)) / h;

                if (t1 >= 0.0f && t1 < 1.0f && t2 >= 0.0f && t2 < 1.0f)
                {
                    disp.x += (1.0f - t1) * (line_r1e.x - line_r1s.x);
                    disp.y += (1.0f - t1) * (line_r1e.y - line_r1s.y);
                    //return true;
                }
            }

            sf::Vector2f pos = r1.getPosition();
            pos.x += disp.x * (shape == 0 ? -1 : +1);
            pos.y += disp.y * (shape == 0 ? -1 : +1);
            r1.setPosition(pos);

        }
    }
    return false;
}


int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(800, 800), "SFML Collisions", sf::Style::Default, settings);
    window.setFramerateLimit(120);

    float rotationAngle = 0.0f;
    float rotationSpeed = 1.0f;
    float movementSpeed = 1.0f;

    float rotationAngle1 = 0.0f;
    float rotationSpeed1 = 1.0f;
    float movementSpeed1 = 1.0f;

    std::vector<RegularPolygon> myConvexShapes;
    myConvexShapes.push_back(RegularPolygon(3, 50));
    myConvexShapes.push_back(RegularPolygon(5, 50));
    myConvexShapes.push_back(RegularPolygon(4, 100));
    myConvexShapes.push_back(RegularPolygon(4, 100));
    myConvexShapes.push_back(RegularPolygon(6, 50));

    myConvexShapes[0].setPosition(static_cast<float>(window.getSize().x * 3 / 4), static_cast<float>(window.getSize().y / 2));
    myConvexShapes[1].setPosition(static_cast<float>(window.getSize().x / 4), static_cast<float>(window.getSize().y / 2));
    myConvexShapes[2].setPosition(600, 600);;
    myConvexShapes[3].setPosition(200, 200);;
    myConvexShapes[4].setPosition(200, 500);;

    //Direction or Indication line
    sf::RectangleShape triangleDirectionLine;
    float distanceToTriangleCentroid = static_cast<float>(std::sqrt(std::pow(myConvexShapes[0].getCentroid().x - myConvexShapes[0].getPoint(2).x, 2) + std::pow(myConvexShapes[0].getCentroid().y - myConvexShapes[0].getPoint(2).y, 2)));
    triangleDirectionLine.setSize(sf::Vector2f(1.0f, distanceToTriangleCentroid));

    //Direction or Indication line
    sf::RectangleShape polygonDirectionLine;
    float distanceToPolygonCentroid = static_cast<float>(std::sqrt(std::pow(myConvexShapes[1].getCentroid().x - myConvexShapes[1].getPoint(3).x, 2) + std::pow(myConvexShapes[1].getCentroid().y - myConvexShapes[1].getPoint(3).y, 2)));
    polygonDirectionLine.setSize(sf::Vector2f(1.0f, distanceToPolygonCentroid));

    //ENUM Movement 
    MovementPolygon movementTriangle = MovementPolygon::NONE;
    MovementPolygon movementPentagon = MovementPolygon::NONE;
    DirectionPolygon directionTriangle = DirectionPolygon::NONE;
    DirectionPolygon directionPentagon = DirectionPolygon::NONE;

    sf::Clock clock;
    sf::Time dt;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            switch (event.type)
            {
            case sf::Event::Closed:
            {
                window.close();
                break;
            }
            case sf::Event::KeyPressed:
            {
                switch (event.key.code)
                {
                case sf::Keyboard::W:
                {
                    movementPentagon = MovementPolygon::FORWARD;
                    break;
                }
                case sf::Keyboard::A:
                {
                    directionPentagon = DirectionPolygon::LEFT;
                    break;
                }
                case sf::Keyboard::S:
                {
                    movementPentagon = MovementPolygon::BACKWARD;
                    break;
                }
                case sf::Keyboard::D:
                {
                    directionPentagon = DirectionPolygon::RIGHT;
                    break;
                }
                case sf::Keyboard::Up:
                {
                    movementTriangle = MovementPolygon::FORWARD;
                    break;
                }
                case sf::Keyboard::Down:
                {
                    movementTriangle = MovementPolygon::BACKWARD;
                    break;
                }
                case sf::Keyboard::Left:
                {
                    directionTriangle = DirectionPolygon::LEFT;
                    break;
                }
                case sf::Keyboard::Right:
                {
                    directionTriangle = DirectionPolygon::RIGHT;
                    break;
                }
                default:
                    break;
                }
                break;
            }
            case sf::Event::KeyReleased:
            {
                switch (event.key.code)
                {
                case sf::Keyboard::W:                
                case sf::Keyboard::S:
                {
                    movementPentagon = MovementPolygon::NONE;
                    break;
                }
                case sf::Keyboard::A:
                case sf::Keyboard::D:
                {
                    directionPentagon = DirectionPolygon::NONE;
                    break;
                }
                case sf::Keyboard::Up:                
                case sf::Keyboard::Down: 
                {
                    movementTriangle = MovementPolygon::NONE;

                    break;
                }
                case sf::Keyboard::Left:               
                case sf::Keyboard::Right:
                {
                    directionTriangle = DirectionPolygon::NONE;
                    break;
                }
                default:
                    break;
                }
                break;
            }         
            default:
                break;
            }
        }
        dt = clock.restart();
        //Pentagon Movement Update
        {
            if (movementPentagon == MovementPolygon::FORWARD)
                myConvexShapes[1].move(movementSpeed1 * calculateDirection(rotationAngle1));
            if (movementPentagon == MovementPolygon::BACKWARD)
                myConvexShapes[1].move(-movementSpeed1 * calculateDirection(rotationAngle1));
            if (directionPentagon == DirectionPolygon::LEFT)
                rotationAngle1 -= rotationSpeed1;
            if (directionPentagon == DirectionPolygon::RIGHT)
                rotationAngle1 += rotationSpeed1;
        }

        //Triangle Movement Update
        {
            if (movementTriangle == MovementPolygon::FORWARD)
                myConvexShapes[0].move(movementSpeed * calculateDirection(rotationAngle));
            if (movementTriangle == MovementPolygon::BACKWARD)
                myConvexShapes[0].move(-movementSpeed * calculateDirection(rotationAngle));
            if (directionTriangle == DirectionPolygon::LEFT)
                rotationAngle -= rotationSpeed;
            if (directionTriangle == DirectionPolygon::RIGHT)
                rotationAngle += rotationSpeed;
        }

        //Polygon direction line
        sf::Vector2f topVertex1 = myConvexShapes[1].getTransform().transformPoint(myConvexShapes[1].getPoint(3));
        polygonDirectionLine.setPosition(topVertex1);
        polygonDirectionLine.setRotation(rotationAngle1);
        myConvexShapes[1].setRotation(rotationAngle1);

        //Triangle direction line
        sf::Vector2f topVertex = myConvexShapes[0].getTransform().transformPoint(myConvexShapes[0].getPoint(2));
        triangleDirectionLine.setPosition(topVertex);
        triangleDirectionLine.setRotation(rotationAngle);
        myConvexShapes[0].setRotation(rotationAngle);

       
        for (int m = 0; m < myConvexShapes.size(); m++)
        {
            for (int n = m + 1; n < myConvexShapes.size(); n++)
            {
                //myConvexShapes[m].overlap |= isSAT(myConvexShapes[m], myConvexShapes[n]);
                //myConvexShapes[m].overlap |= isSAT_STATIC(myConvexShapes[m], myConvexShapes[n]);
                //myConvexShapes[m].overlap |= isDIAG(myConvexShapes[m], myConvexShapes[n]);
                myConvexShapes[m].overlap |= isDIAG_STATIC(myConvexShapes[m], myConvexShapes[n]);
            }
        }

        window.clear();
        
        for (auto& i : myConvexShapes)
        {
            if (i.overlap)
                i.setOutlineColor(sf::Color::Red);
            else 
                i.setOutlineColor(sf::Color::White);

            window.draw(i);
            i.overlap = false;
        }
        window.draw(triangleDirectionLine);
        window.draw(polygonDirectionLine);

        window.display();
    }
    return 0;
}