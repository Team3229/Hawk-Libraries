param (
    [string]$version,
    [string]$commitMessage
)

# Update version in publish.gradle
$content = Get-Content publish.gradle
$content = $content -replace 'def pubVersion = ".*"', "def pubVersion = `"$version`""
Set-Content publish.gradle -Value $content

# Update version in Hawk-Libraries.json
$content = Get-Content Hawk-Libraries.json
$replacement = '"version": "' + $version + '"'
$content = $content -replace '"version": ".*"', $replacement
Set-Content Hawk-Libraries.json -Value $content

# Run Gradle publish
./gradlew publish

# Commit and push changes
git add -A
git commit -m $commitMessage
git push