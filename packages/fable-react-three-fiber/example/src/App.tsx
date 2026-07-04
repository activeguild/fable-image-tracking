import { useMemo, useState } from 'react';
import { CanvasTexture, SRGBColorSpace } from 'three';
import { FableCanvas, FableCamera, ImageTracker, PlanarContent } from '@j1ngzoue/fable-react-three-fiber';
import { createSampleTargetCanvas } from './sampleTarget';
import { createSampleImageCanvas } from './sampleContent';

export function App() {
  const [status, setStatus] = useState('initializing');
  const [placement, setPlacement] = useState<'on' | 'side' | 'both'>('on');
  // The sample target is procedural; a real app passes an image URL instead:
  //   <ImageTracker targetImage="/my-target.png">
  const target = useMemo(() => createSampleTargetCanvas(384), []);
  const contentTexture = useMemo(() => {
    const texture = new CanvasTexture(createSampleImageCanvas());
    texture.colorSpace = SRGBColorSpace;
    return texture;
  }, []);

  return (
    <>
      <FableCanvas
        style={{ width: '100vw', height: '100vh' }}
        onReady={() => setStatus('searching')}
        onError={(err) => setStatus(`error: ${err.message}`)}
      >
        <FableCamera />
        {/* Zappar-style: the tracker owns the target image (a plain image,
            no .zpt training file), and visibility callbacks get an anchor. */}
        <ImageTracker
          targetImage={target}
          onVisible={() => setStatus('tracking')}
          onNotVisible={() => setStatus('searching')}
        >
          {/* PlanarContent is a homography-pinned plane mesh: pass any
              material as children. Units are scene units (target height =
              2); this square target is 2x2, so offset x=2.3 sits beside it. */}
          {placement !== 'side' && (
            <PlanarContent>
              <meshBasicMaterial map={contentTexture} toneMapped={false} />
            </PlanarContent>
          )}
          {placement !== 'on' && (
            <PlanarContent offset={{ x: 2.3 }}>
              <meshBasicMaterial map={contentTexture} toneMapped={false} />
            </PlanarContent>
          )}
          {/* Zappar-compatible units (default): the target is 2 units tall,
              so a 0.6 cube is 30% of this square marker. */}
          <mesh position={[0, 0, 0.3]}>
            <boxGeometry args={[0.6, 0.6, 0.6]} />
            <meshStandardMaterial color="hotpink" />
          </mesh>
        </ImageTracker>
        <hemisphereLight args={[0xffffff, 0x555566, 2.2]} />
        <directionalLight position={[1, 2, 3]} intensity={1.5} />
      </FableCanvas>
      <div
        style={{
          position: 'fixed',
          left: 12,
          bottom: 12,
          display: 'flex',
          gap: 10,
          alignItems: 'center',
          font: '13px system-ui',
        }}
      >
        <div
          id="status"
          style={{ color: '#fff', background: 'rgba(0,0,0,0.5)', padding: '6px 10px', borderRadius: 6 }}
        >
          {status}
        </div>
        <select
          id="placement-select"
          value={placement}
          onChange={(e) => setPlacement(e.target.value as 'on' | 'side' | 'both')}
          style={{
            background: '#1f2937',
            color: '#e5e7eb',
            border: '1px solid #4b5563',
            borderRadius: 6,
            padding: '5px 8px',
            fontSize: 13,
          }}
        >
          <option value="on">画像（マーカー上）</option>
          <option value="side">画像（マーカー横）</option>
          <option value="both">画像（両方）</option>
        </select>
      </div>
    </>
  );
}
