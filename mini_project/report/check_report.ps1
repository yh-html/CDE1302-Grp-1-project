# Checks the group report against the mini-project brief's format rules, using Word itself.
#   .\check_report.ps1                      checks CDE1302_Mini_Project_Report.docx here
#   .\check_report.ps1 -Path other.docx -Pdf   also writes a PDF next to it
# Rules (brief, section 6): at most 10 pages including cover and appendix, single column,
# single spacing, Times New Roman at least 12 pt, GitHub link, submit within 48 h of the demo.
param(
    [string]$Path = (Join-Path $PSScriptRoot 'CDE1302_Mini_Project_Report.docx'),
    [switch]$Pdf
)
$ErrorActionPreference = 'Stop'
$Path = (Resolve-Path $Path).Path
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0
$fail = 0; $warn = 0
function Fail($m) { $script:fail++; Write-Host "  [FAIL] $m" -ForegroundColor Red }
function Warn($m) { $script:warn++; Write-Host "  [WARN] $m" -ForegroundColor Yellow }
function Ok($m)   { Write-Host "  [ OK ] $m" -ForegroundColor Green }
try {
    $doc = $word.Documents.Open($Path, $false, $true)       # read-only
    $doc.Repaginate()

    $pages = $doc.ComputeStatistics(2)                      # wdStatisticPages
    Write-Host "Report: $Path"
    if ($pages -gt 10) { Fail "$pages pages (limit 10, including cover and appendix)" }
    elseif ($pages -eq 10) { Warn "$pages pages: at the limit, no room to add anything" }
    else { Ok "$pages pages (limit 10)" }

    $cols = $doc.PageSetup.TextColumns.Count
    if ($cols -ne 1) { Fail "$cols text columns (must be single column)" } else { Ok "single column" }

    $badFont = @{}; $small = 0; $multi = 0; $total = 0
    foreach ($p in $doc.Paragraphs) {
        $total++
        $r = $p.Range
        # skip empty paragraphs and Word's invisible end-of-cell / end-of-row marks (CR, BEL)
        if (($r.Text -replace '[\s\x07]', '').Length -eq 0) { continue }
        $name = $r.Font.Name
        if ($name -ne 'Times New Roman') { $badFont[$name] = 1 + [int]$badFont[$name] }
        $sz = $r.Font.Size
        # 9999999 means mixed sizes in the paragraph; check the runs then
        if ($sz -eq 9999999) { foreach ($w in $r.Words) { if ($w.Font.Size -lt 12 -and $w.Text.Trim().Length) { $small++; break } } }
        elseif ($sz -lt 12) { $small++ }
        if ($p.Format.LineSpacingRule -ne 0) { $multi++ }   # 0 = wdLineSpaceSingle
    }
    if ($badFont.Count) { Fail ("non Times New Roman paragraphs: " + (($badFont.GetEnumerator() | ForEach-Object { "$($_.Key) x$($_.Value)" }) -join ', ')) }
    else { Ok "all text Times New Roman" }
    if ($small) { Fail "$small paragraph(s) with text under 12 pt" } else { Ok "no text under 12 pt" }
    if ($multi) { Fail "$multi paragraph(s) not single-spaced" } else { Ok "single spacing" }

    $text = $doc.Content.Text
    $todo = [regex]::Matches($text, '\[[^\]\r]{0,300}\]').Count
    if ($todo -gt 0) { Warn "$todo bracketed placeholder(s) still to fill (highlighted yellow in the file)" } else { Ok "no placeholders left" }
    if ($text -match 'github\.com/[\w.-]+/[\w.-]+') { Ok "GitHub link present: $($Matches[0])" } else { Warn "no GitHub repository link found" }
    Write-Host ("Words: {0}   Figures/tables: {1} pictures, {2} tables" -f $doc.ComputeStatistics(0), $doc.InlineShapes.Count, $doc.Tables.Count)

    if ($Pdf) {
        $pdfPath = [IO.Path]::ChangeExtension($Path, '.pdf')
        $doc.ExportAsFixedFormat($pdfPath, 17)              # wdExportFormatPDF
        Write-Host "PDF written: $pdfPath"
    }
    $doc.Close($false)
}
finally {
    $word.Quit()
    [void][Runtime.InteropServices.Marshal]::ReleaseComObject($word)
}
Write-Host ("Result: {0} FAIL, {1} WARN" -f $fail, $warn)
exit ([int]($fail -gt 0))
