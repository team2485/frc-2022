package frc.util.photonVision;

/*
 * Autogenerated file! Do not manually edit this file. This version is regenerated
 * any time the publish task is run, or when this file is deleted.
 */

import java.util.regex.Matcher;
import java.util.regex.Pattern;

@SuppressWarnings("ALL")
public final class PhotonVersion {
  public static final String versionString = "${version}";
  public static final String buildDate = "${date}";
  public static final boolean isRelease = !versionString.startsWith("dev");

  public static final boolean versionMatches(String other) {
    String c = versionString;
    Pattern p = Pattern.compile("v[0-9]+.[0-9]+.[0-9]+");
    Matcher m = p.matcher(c);
    if (m.find()) {
      c = m.group(0);
    } else {
      return false;
    }
    m = p.matcher(other);
    if (m.find()) {
      other = m.group(0);
    } else {
      return false;
    }
    return c.equals(other);
  }
}