package frc.alotobots.library.vision.photonvision.objectdetection;

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

  public String getName() {
    return name;
  }

  public double getLength() {
    return length;
  }

  public double getWidth() {
    return width;
  }

  public double getHeight() {
    return height;
  }

}
