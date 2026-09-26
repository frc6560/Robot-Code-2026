// Run with: npx tsx tools/bline-preview.ts BLINE_WEB_SOURCE PATH CONFIG OUTPUT_DIR
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';

async function main() {
  const [source, input, configFile, output] = process.argv.slice(2);
  if (!output) throw new Error('Expected source directory, path JSON, config JSON, output directory');
  const { deserializePath } = await import(pathToFileURL(path.resolve(source, 'src/core/io/projectSerde.ts')).href);
  const { simulatePath } = await import(pathToFileURL(path.resolve(source, 'src/core/sim/simulatePath.ts')).href);
  const { createProjectConfig, projectConfigDefaultLookup } = await import(pathToFileURL(path.resolve(source, 'src/core/config/projectConfig.ts')).href);
  const raw = fs.readFileSync(input, 'utf8');
  const rawConfig = fs.readFileSync(configFile, 'utf8');
  const config = createProjectConfig(JSON.parse(rawConfig));
  const model = deserializePath(JSON.parse(raw), projectConfigDefaultLookup(config));
  const result = simulatePath(model, config, { dt_s: 0.02 });
  fs.mkdirSync(output, { recursive: true });
  const rows = result.times_sorted.map((t: number) => [t, ...result.poses_by_time.get(t)]);
  fs.writeFileSync(path.join(output, 'gui-preview.csv'), 'time_seconds,x_meters,y_meters,heading_radians\n' + rows.map((r: number[]) => r.join(',')).join('\n') + '\n');
  const metadata = { source_revision: execFileSync('git', ['-C', source, 'rev-parse', 'HEAD'], { encoding: 'utf8' }).trim(), path: path.resolve(input), config: path.resolve(configFile), path_sha256: createHash('sha256').update(raw).digest('hex'), config_sha256: createHash('sha256').update(rawConfig).digest('hex'), dt_seconds: 0.02, samples: rows.length, duration_seconds: result.total_time_s, start: rows[0], end: rows.at(-1) };
  fs.writeFileSync(path.join(output, 'preview-metadata.json'), JSON.stringify(metadata, null, 2) + '\n');
  console.log(metadata);
}
main().catch(error => { console.error(error); process.exitCode = 1; });
