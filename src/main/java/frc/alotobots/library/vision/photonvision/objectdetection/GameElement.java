package frc.alotobots.library.vision.photonvision.objectdetection;

/**
 * Represents a physical game element that can be detected by the vision system. Stores the
 * element's name and physical dimensions.
 */
public class GameElement {

  private String name;
  private double length;
  private double width;
  private double height;

  /**
   * Represents a game element (measurements are in meters)
   *
   * @param name The name of the object
   * @param length The length of the object
   * @param width The width of the object
   * @param height The height of the object (floor to top)
   */
  public GameElement(String name, double length, double width, double height) {
    this.name = name;
    this.length = length;
    this.width = width;
    this.height = height;
  }

  /**
   * Gets the name identifier of this game element.
   *
   * @return The name of the game element
   */
  public String getName() {
    return name;
  }

  /**
   * Gets the length dimension of this game element.
   *
   * @return The length in meters
   */
  public double getLength() {
    return length;
  }

  /**
   * Gets the width dimension of this game element.
   *
   * @return The width in meters
   */
  public double getWidth() {
    return width;
  }

  /**
   * Gets the height dimension of this game element.
   *
   * @return The height in meters from floor to top
   */
  public double getHeight() {
    return height;
  }
}
