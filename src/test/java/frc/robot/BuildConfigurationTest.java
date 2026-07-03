package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Verifies the build.gradle configuration, specifically that the com.diffplug.spotless plugin is
 * pinned to the expected version. This acts as a regression guard against accidental version
 * downgrades or edits.
 */
class BuildConfigurationTest {

    private static String buildGradleContent;

    @BeforeAll
    static void loadBuildGradle() throws IOException {
        // Resolve build.gradle relative to the project root. When tests run via
        // `./gradlew test` the working directory is the project root.
        Path buildFile = Paths.get("build.gradle");
        assertTrue(Files.exists(buildFile), "build.gradle must exist at project root");
        buildGradleContent = Files.readString(buildFile);
    }

    // -----------------------------------------------------------------------
    // Spotless plugin version tests
    // -----------------------------------------------------------------------

    @Test
    void spotlessPluginVersionIs_8_7_0() {
        assertTrue(
                buildGradleContent.contains("\"com.diffplug.spotless\" version \"8.7.0\""),
                "build.gradle must declare com.diffplug.spotless version 8.7.0");
    }

    @Test
    void spotlessPluginVersionIsNot_8_4_0() {
        // Regression: ensure the old (pre-bump) version is no longer referenced.
        assertFalse(
                buildGradleContent.contains("\"com.diffplug.spotless\" version \"8.4.0\""),
                "build.gradle must NOT reference the old spotless version 8.4.0");
    }

    @Test
    void spotlessPluginIdIsPresent() {
        assertTrue(
                buildGradleContent.contains("id \"com.diffplug.spotless\""),
                "build.gradle must declare the com.diffplug.spotless plugin id");
    }

    @Test
    void exactSpotlessDeclarationLineIsPresent() {
        // Validate the full declaration as it appears in the plugins block.
        String expectedLine = "id \"com.diffplug.spotless\" version \"8.7.0\"";
        assertTrue(
                buildGradleContent.contains(expectedLine),
                "build.gradle must contain the exact declaration: " + expectedLine);
    }

    // -----------------------------------------------------------------------
    // Boundary / negative tests
    // -----------------------------------------------------------------------

    @Test
    void noOlderSpotlessMajorMinorVersionsPresent() {
        // Guard against any leftover references to 8.4.x or earlier 8.x builds
        // that were superseded by this bump.
        String[] oldVersionPrefixes = {"version \"8.0.", "version \"8.1.", "version \"8.2.",
                "version \"8.3.", "version \"8.4.", "version \"8.5.", "version \"8.6."};
        for (String prefix : oldVersionPrefixes) {
            assertFalse(
                    buildGradleContent.contains("\"com.diffplug.spotless\" " + prefix),
                    "build.gradle must not contain an older spotless version matching: " + prefix);
        }
    }

    @Test
    void spotlessVersionStringFormatIsValid() {
        // The version must follow semantic versioning (digits separated by dots).
        String marker = "\"com.diffplug.spotless\" version \"";
        int idx = buildGradleContent.indexOf(marker);
        assertTrue(idx >= 0, "Spotless plugin declaration must be present in build.gradle");

        int versionStart = idx + marker.length();
        int versionEnd = buildGradleContent.indexOf("\"", versionStart);
        assertTrue(versionEnd > versionStart, "Spotless version string must be closed with a quote");

        String version = buildGradleContent.substring(versionStart, versionEnd);
        assertTrue(
                version.matches("\\d+\\.\\d+\\.\\d+"),
                "Spotless version must follow MAJOR.MINOR.PATCH format, got: " + version);
    }

    @Test
    void onlyOneSpotlessPluginDeclarationExists() {
        // Ensure there is exactly one spotless plugin declaration (no duplicates).
        String marker = "\"com.diffplug.spotless\"";
        int firstIdx = buildGradleContent.indexOf(marker);
        assertTrue(firstIdx >= 0, "At least one spotless plugin declaration must exist");

        // The second occurrence should only be the spotless { } configuration block,
        // not a second id/version declaration.
        int secondIdx = buildGradleContent.indexOf(marker, firstIdx + 1);
        if (secondIdx >= 0) {
            // If a second occurrence exists it must NOT be an `id` declaration.
            String surroundingText =
                    buildGradleContent.substring(Math.max(0, secondIdx - 10), secondIdx);
            assertFalse(
                    surroundingText.trim().endsWith("id"),
                    "There must not be a duplicate `id` declaration for com.diffplug.spotless");
        }
    }
}
