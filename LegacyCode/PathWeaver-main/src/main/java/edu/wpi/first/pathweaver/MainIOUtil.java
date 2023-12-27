package edu.wpi.first.pathweaver;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Stream;

import javafx.scene.control.TreeItem;

public final class MainIOUtil {
  private static final Logger LOGGER = Logger.getLogger(MainIOUtil.class.getName());

  private MainIOUtil() {
  }

  /**
   * Creates treeItem for every file in directory.
   *
   * @param directory absolute location of directory
   * @param root      Treeitem to add all new items as children
   */
  public static void setupItemsInDirectory(String directory, TreeItem<String> root) {
    File folder = new File(directory);
    if (!folder.exists()) {
      folder.mkdir();
    }
    String[] listOfFiles = folder.list();
    for (String name : listOfFiles) {
      addChild(root, name);
    }
  }

  /**
   * Checks if a file exists with filename if so appends incremental value.
   *
   * @param directory The directory the file will be saved to
   * @param pathName The preferred filename
   * @param extension The file extension
   *
   * @return the next valid filename for the path name
   */
  public static String getValidFileName(String directory, String pathName, String extension) {
    // remove the _number following the file name
    String nameNoVersion = pathName.replaceFirst("_[0-9]+", "");
    File file = new File(directory, nameNoVersion + extension);
    for (int num = 0; file.exists(); num++) {
      file = new File(directory, nameNoVersion + "_" + num + extension);
    }
    return file.getName();
  }

  /**
   * Checks if the rename is valid.
   *
   * @param directory The directory the file will be saved
   * @param oldName   The old name of the file
   * @param newName   The desired new name of the file
   *
   * @return true if the rename is allowed
   */
  public static boolean isValidRename(String directory, String oldName, String newName) {
    if (oldName.equals(newName)) { //no name change is always valid
      return true;
    } else {
      //if newName is already a valid name return true
      String validName = MainIOUtil.getValidFileName(directory, newName, "");
      return newName.equals(validName);
    }
  }

  /**
   * Rename file belonging to a TreeItem with new name.
   *
   * @param directory The directory the file is in
   * @param item      The treeitem that the file belongs to
   * @param newName   The new name of the file
   */
  public static void rename(String directory, TreeItem<String> item, String newName) {
    File oldFile = new File(directory, item.getValue());
    File newFile = new File(directory, newName);

    if (oldFile.renameTo(newFile)) {
      //success
    } else if (newFile.exists()) {
      LOGGER.log(Level.WARNING, "Could not rename "
          + newFile.getAbsolutePath() + " already exists");
    } else if (oldFile.exists()) {
      LOGGER.log(Level.WARNING, "Could not rename , unknown error");
    } else {
      LOGGER.log(Level.WARNING, "Could not rename "
          + oldFile.getAbsolutePath() + " doesnt exist");
    }
  }

  /**
   * Create new tree item and add it as child to root.
   *
   * @param root Root of new treeItem.
   * @param name String name for new treeItem.
   *
   * @return the TreeItem created
   */
  public static TreeItem<String> addChild(TreeItem<String> root, String name) {
    TreeItem<String> item = new TreeItem<>(name);
    root.getChildren().add(item);
    return item;
  }

  /**
   * Delete item and file associated with it.
   *
   * @param directory Location of file
   * @param item      Item to delete
   */
  public static void deleteItem(String directory, TreeItem<String> item) {

    File itemFile = new File(directory + item.getValue());
    if (itemFile.exists()) {
      if (itemFile.delete()) {
        item.getParent().getChildren().remove(item);
      } else {
        LOGGER.log(Level.WARNING, "Could not delete file: " + itemFile.getAbsolutePath());
      }
    } else {
      LOGGER.log(Level.WARNING, "Could not find file to delete: " + itemFile.getAbsolutePath());
    }
  }

  /**
   * Load auton from file.
   *
   * @param location Directory of file
   * @param filename Name of auton file
   * @param root     Auton treeItem to add new created items to.
   */
  public static void loadAuton(String location, String filename, TreeItem<String> root) {
    root.getChildren().clear();
    try(BufferedReader reader = Files.newBufferedReader(Paths.get(location, filename))) {
      String line = reader.readLine();
      while (line != null) {
        addChild(root, line);
        line = reader.readLine();
      }
      reader.close();
    } catch (IOException e) {
      LOGGER.log(Level.WARNING, "Could not load auton file", e);
    }
  }

  /**
   * Save auton to its file.
   *
   * @param location Directory to save auton file
   * @param filename Name of new auton file
   * @param root     Auton treeItem to save
   */
  public static void saveAuton(String location, String filename, TreeItem<String> root) {
    try(BufferedWriter writer = Files.newBufferedWriter(Paths.get(location, filename))) {
      for (TreeItem<String> item : root.getChildren()) {
        writer.write(item.getValue());
        writer.newLine();
      }
    } catch (IOException e) {
      LOGGER.log(Level.WARNING, "Could not save auton file", e);
    }
  }

  public static void copyGroupFiles(String autonDirectory, String groupDirectory) throws IOException {
    File folder = new File(groupDirectory + "/");
    File autonfolder = new File(autonDirectory);
    if (folder.exists()) {
      if (!autonfolder.exists()) {
        autonfolder.mkdir();
      }
      try (Stream<Path> stream = Files.walk(folder.toPath(), 1)) {
        stream.filter(Files::isRegularFile).forEach(
                source -> copy(source, Paths.get(autonDirectory + source.getFileName())));
      }
    }
  }

  private static void copy(Path source, Path dest) {
    try {
        Files.copy(source, dest, StandardCopyOption.REPLACE_EXISTING);
    } catch (IOException e) {
        LOGGER.log(Level.WARNING, "Could not copy file", e);
    }
  }
}
