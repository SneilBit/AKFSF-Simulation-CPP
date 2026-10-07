# System Includes
import math
import os

import pygame


def string_format(fmt, *args):
    return fmt % args


class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


def _is_dataset(points):
    # Emulates the C++ overloads for std::vector<Vector2> and std::vector<std::vector<Vector2>>
    return len(points) > 0 and isinstance(points[0], (list, tuple))


def _as_vector2(point):
    return point if isinstance(point, Vector2) else Vector2(point[0], point[1])


def _to_c_int(value):
    # C++ double -> int conversion (truncation toward zero). Out of range / NaN values are
    # undefined behaviour in C++; on x86 they become INT_MIN, which is reproduced here.
    if not math.isfinite(value) or value >= 2147483648.0 or value <= -2147483649.0:
        return -2147483648
    return int(value)


def transformPoints(points, position, rotation):
    if _is_dataset(points):
        transformedDataset = []
        for p in points:
            transformedDataset.append(transformPoints(p, position, rotation))
        return transformedDataset

    ctheta = math.cos(rotation)
    stheta = math.sin(rotation)
    transformedPoints = []
    for point in points:
        point = _as_vector2(point)
        x = point.x * ctheta - stheta * point.y + position.x
        y = point.x * stheta + ctheta * point.y + position.y
        transformedPoints.append(Vector2(x, y))
    return transformedPoints


def offsetPoints(points, offset):
    if _is_dataset(points):
        transformedDataset = []
        for p in points:
            transformedDataset.append(offsetPoints(p, offset))
        return transformedDataset

    transformedPoints = []
    for point in points:
        point = _as_vector2(point)
        transformedPoints.append(Vector2(point.x + offset.x, point.y + offset.y))
    return transformedPoints


class Display:

    def __init__(self):
        self.mScreenWidth = 0
        self.mScreenHeight = 0
        self.mViewWidth = 0.0
        self.mViewHeight = 0.0
        self.mViewXOffset = 0.0
        self.mViewYOffset = 0.0
        self.mWindow = None
        self.mRenderer = None
        self.mMainFont = None
        self.mDrawColour = (0, 0, 0, 0xFF)

    def __del__(self):
        self.destroyRenderer()

    def createRenderer(self, title, screenWidth, screenHeight):
        self.destroyRenderer()

        self.mScreenWidth = screenWidth
        self.mScreenHeight = screenHeight

        self.mViewWidth = self.mScreenWidth
        self.mViewHeight = self.mScreenHeight
        self.mViewXOffset = 0.0
        self.mViewYOffset = 0.0

        # Create window
        try:
            self.mWindow = pygame.display.set_mode((screenWidth, screenHeight))
            pygame.display.set_caption(title)
        except pygame.error as e:
            print("Window could not be created! SDL_Error: " + str(e))
            self.destroyRenderer()
            return False

        # Create Renderer
        # (pygame draws directly onto the window surface; SDL_RENDERER_PRESENTVSYNC is
        # emulated by limiting the frame rate in showScreen())
        self.mRenderer = self.mWindow
        self.mClock = pygame.time.Clock()

        # Create Font
        font_path = "Roboto-Regular.ttf"
        if not os.path.exists(font_path):
            font_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "Roboto-Regular.ttf")
        try:
            self.mMainFont = pygame.font.Font(font_path, 18)
        except (pygame.error, OSError) as e:
            print("Failed to load font! SDL_ttf Error: " + str(e))
            self.destroyRenderer()
            return False

        return True

    def destroyRenderer(self):
        # Destroy Renderer
        if self.mRenderer is not None:
            self.mRenderer = None

        # Destroy Window
        if self.mWindow is not None:
            self.mWindow = None
            pygame.display.quit()

        # Destroy Font
        if self.mMainFont is not None:
            self.mMainFont = None

    def clearScreen(self):
        if self.mRenderer is not None:
            # Clear screen
            self.mRenderer.fill((0x00, 0x00, 0x00, 0xFF))

    def showScreen(self):
        if self.mRenderer is not None:
            pygame.display.flip()
            self.mClock.tick(60)

    def getScreenWidth(self):
        return float(self.mScreenWidth)

    def getScreenHeight(self):
        return float(self.mScreenHeight)

    def getScreenAspectRatio(self):
        return self.getScreenWidth() / self.getScreenHeight()

    def setView(self, *args):
        if len(args) == 4:
            width, height, xOffset, yOffset = args
            self.mViewWidth = math.fabs(width)
            self.mViewHeight = math.fabs(height)
        else:
            xOffset, yOffset = args
        self.mViewXOffset = xOffset - self.mViewHeight / 2.0
        self.mViewYOffset = yOffset - self.mViewWidth / 2.0

    def setDrawColour(self, red, green, blue, alpha=0xFF):
        self.mDrawColour = (red, green, blue, alpha)

    def drawLine(self, startPos, endPos):
        p1 = self.transformPoint(startPos)
        p2 = self.transformPoint(endPos)
        pygame.draw.line(self.mRenderer, self.mDrawColour,
                         (_to_c_int(p1.x), _to_c_int(p1.y)), (_to_c_int(p2.x), _to_c_int(p2.y)))

    def drawLines(self, points):
        if _is_dataset(points):
            for p in points:
                self.drawLines(p)
            return
        for i in range(1, len(points)):
            self.drawLine(points[i - 1], points[i])

    def transformPoint(self, point):
        point = _as_vector2(point)
        dx = point.x - self.mViewXOffset
        dy = point.y - self.mViewYOffset
        y = self.mScreenHeight - (dx / self.mViewHeight) * self.mScreenHeight
        x = (dy / self.mViewWidth) * self.mScreenWidth
        return Vector2(x, y)

    def drawText_MainFont(self, text, pos, scale=1, color=(0, 0, 0), centered=False):
        try:
            surface = self.mMainFont.render(text, True, color)
        except pygame.error as e:
            print("Unable to render text surface! SDL_ttf Error: " + str(e))
            return

        width = int(surface.get_width() * scale)
        height = int(surface.get_height() * scale)
        x = int(pos.x)
        y = int(pos.y)

        if centered:
            x -= int(width / 2)
            y -= int(height / 2)

        if (width, height) != surface.get_size():
            surface = pygame.transform.scale(surface, (max(width, 0), max(height, 0)))
        self.mRenderer.blit(surface, (x, y))
