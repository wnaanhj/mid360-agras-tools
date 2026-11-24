# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 只使用JDK编译独立Java工具，不需要Android SDK，不操作手机。
$ErrorActionPreference='Stop'
$repo=Split-Path -Parent $PSScriptRoot
$classes=Join-Path $repo 'build/java'
$javac=Get-Command javac -ErrorAction SilentlyContinue
$java=Get-Command java -ErrorAction SilentlyContinue
if(!$javac -or !$java){throw 'Install JDK 17 and put its bin directory on PATH.'}
New-Item -ItemType Directory -Force $classes | Out-Null
$sources=@(Get-ChildItem -LiteralPath (Join-Path $repo 'java/src'),(Join-Path $repo 'java/test') -Recurse -Filter '*.java' | ForEach-Object FullName)
& $javac.Source --release 11 -encoding UTF-8 -d $classes @sources
if($LASTEXITCODE){throw "javac failed: $LASTEXITCODE"}
& $java.Source '-Dfile.encoding=UTF-8' -cp $classes cn.t100.mapping.ToolsCheck
if($LASTEXITCODE){throw "ToolsCheck failed: $LASTEXITCODE"}
